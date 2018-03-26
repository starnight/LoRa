/*-
 * Copyright (c) 2018 Jian-Hong, Pan <starnight@g.ncu.edu.tw>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>

#include "lora.h"
#include "lorawan.h"
#include "lrwsec.h"

static void rx_timeout_work(struct work_struct *work);

struct lrw_session *
lrw_alloc_ss(struct lrw_struct *lrw_st)
{
	struct lrw_session *ss;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	ss = kzalloc(sizeof(struct lrw_session), GFP_KERNEL);
	if (!ss)
		goto lrw_alloc_ss_end;
	
	ss->lrw_st = lrw_st;
	memcpy(ss->devaddr, lrw_st->devaddr, LRW_DEVADDR_LEN);
	ss->appkey = lrw_st->appkey;
	ss->nwkskey = lrw_st->nwkskey;
	ss->appskey = lrw_st->appskey;
	INIT_LIST_HEAD(&ss->entry);

	ss->tx_should_ack = false;
	ss->retry = 3;
	spin_lock_init(&ss->state_lock);
	INIT_WORK(&ss->timeout_work, rx_timeout_work);

lrw_alloc_ss_end:
	return ss;
}

void
lrw_free_ss(struct lrw_session *ss)
{
	kfree_skb(ss->tx_skb);
	kfree_skb(ss->rx_skb);

	kfree(ss);
}

void
lrw_del_ss(struct lrw_session *ss)
{
	list_del(&ss->entry);
	lrw_free_ss(ss);
}

void
lrw_del_all_ss(struct lrw_struct *lrw_st)
{
	struct lrw_session *ss, *tmp;

	mutex_lock(&lrw_st->ss_list_lock);
	lrw_st->_cur_ss = NULL;
	list_for_each_entry_safe(ss, tmp, &lrw_st->ss_list, entry) {
		del_timer(&ss->timer);
		lrw_del_ss(ss);
	}
	mutex_unlock(&lrw_st->ss_list_lock);
}

int
lora_start_hw(struct lrw_struct *lrw_st)
{
	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);
	lrw_st->nwks_shash_tfm = lrw_mic_key_setup(lrw_st->nwkskey,
						   LORA_KEY_LEN);
	lrw_st->nwks_skc_tfm = lrw_encrypt_key_setup(lrw_st->nwkskey,
						     LORA_KEY_LEN);
	lrw_st->apps_skc_tfm = lrw_encrypt_key_setup(lrw_st->appskey,
						     LORA_KEY_LEN);
	lrw_st->ops->start(&lrw_st->hw);
	lrw_st->state = LORA_START;

	return 0;
}

void
lora_stop_hw(struct lrw_struct *lrw_st)
{
	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);
	lrw_st->state = LORA_STOP;
	pr_debug("%s: going to stop hardware\n", LORAWAN_MODULE_NAME);
	lrw_st->ops->stop(&lrw_st->hw);

	pr_debug("%s: going to kill tasks & flush works", LORAWAN_MODULE_NAME);
	tasklet_kill(&lrw_st->xmit_task);
	flush_work(&lrw_st->rx_work);

	pr_debug("%s: going to delete all session\n", LORAWAN_MODULE_NAME);
	lrw_del_all_ss(lrw_st);

	pr_debug("%s: going to free mic tfm\n", LORAWAN_MODULE_NAME);
	lrw_mic_key_free(lrw_st->nwks_shash_tfm);
	pr_debug("%s: going to free nwks tfm\n", LORAWAN_MODULE_NAME);
	lrw_encrypt_key_free(lrw_st->nwks_skc_tfm);
	pr_debug("%s: going to free apps tfm\n", LORAWAN_MODULE_NAME);
	lrw_encrypt_key_free(lrw_st->apps_skc_tfm);
}

void
lrw_prepare_tx_frame(struct lrw_session *ss)
{
	struct lrw_struct *lrw_st = ss->lrw_st;
	struct sk_buff *skb = ss->tx_skb;
	u8 mhdr, fctrl, fport;
	u8 mic[4];

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	mhdr = LRW_UNCONFIRMED_DATA_UP << 5;
	if ((mhdr & (0x6 << 5)) == (0x4 << 5))
		ss->tx_should_ack = true;

	fctrl = 0;
	if (lrw_st->rx_should_ack) {
		fctrl |= 0x20;
		lrw_st->rx_should_ack = false;
	}

	/* Encrypt the plain buffer content */
	lrw_encrypt_buf(lrw_st->apps_skc_tfm, LRW_UPLINK,
			ss->devaddr, ss->fcnt_up, skb->data, skb->len);

	/* Push FPort */
	if (skb->len) {
		fport = ss->fport;
		memcpy(skb_push(skb, LRW_FPORT_LEN), &fport, LRW_FPORT_LEN);
	}

	/* Push FCnt_Up */
	memcpy(skb_push(skb, 2), &ss->fcnt_up, 2);

	/* Push FCtrl */
	memcpy(skb_push(skb, 1), &fctrl, 1);

	/* Push DevAddr */
	memcpy(skb_push(skb, LRW_DEVADDR_LEN), ss->devaddr, LRW_DEVADDR_LEN);

	/* Push MHDR */
	memcpy(skb_push(skb, LRW_MHDR_LEN), &mhdr, LRW_MHDR_LEN);

	/* Put MIC */
	lrw_calc_mic(lrw_st->nwks_shash_tfm, LRW_UPLINK,
		     ss->devaddr, ss->fcnt_up, skb->data, skb->len, mic);
	memcpy(skb_put(skb, LRW_MIC_LEN), mic, LRW_MIC_LEN);
}

void
lrw_xmit(unsigned long data)
{
	struct lrw_struct *lrw_st = (struct lrw_struct *) data;
	struct lrw_session *ss = lrw_st->_cur_ss;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);
	ss->state = LRW_XMITTING_SS;
	lrw_st->ops->xmit_async(&lrw_st->hw, ss->tx_skb);
}

void
lrw_parse_frame(struct lrw_session *ss, struct sk_buff *skb)
{
	struct lrw_fhdr *fhdr = &ss->rx_fhdr;
	__le16 *p_fcnt;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	/* Get message type */
	fhdr->mtype = skb->data[0];
	skb_pull(skb, LRW_MHDR_LEN);

	/* Trim Device Address */
	skb_pull(skb, 4);

	/* Get frame control */
	fhdr->fctrl = skb->data[0];
	skb_pull(skb, 1);

	/* Ack the original TX frame if it should be acked */
	if (ss->tx_should_ack && (fhdr->fctrl & 0x20))
		ss->tx_should_ack = false;

	/* Get frame count */
	p_fcnt = (__le16 *)skb->data;
	fhdr->fcnt = le16_to_cpu(*p_fcnt);
	skb_pull(skb, 2);

	/* Get frame options */
	fhdr->fopts_len = fhdr->fctrl & 0xF;
	if (fhdr->fopts_len > 0) {
		memcpy(fhdr->fopts, skb->data, fhdr->fopts_len);
		skb_pull(skb, fhdr->fopts_len);
	}

	// TODO: Parse frame options

	/* Remove message integrity code */
	skb_trim(skb, skb->len - LRW_MIC_LEN);
}

struct lrw_session *
lrw_rx_skb_2_session(struct lrw_struct *lrw_st, struct sk_buff *rx_skb)
{
	struct lrw_session *ss;
	u16 fcnt;
	__le16 *p_fcnt;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	p_fcnt = (__le16 *)(rx_skb->data + 6);
	fcnt = le16_to_cpu(*p_fcnt);

	/* Find the corresponding session */
	ss = lrw_st->_cur_ss;

	/* Frame count down check */
	if (fcnt > (ss->fcnt_down & 0xFFFF))
		ss->rx_skb = rx_skb;
	else
		ss = NULL;

	return ss;
}

void
lrw_rx_work(struct work_struct *work)
{
	struct lrw_struct *lrw_st;
	struct lrw_session *ss;
	struct sk_buff *skb;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	lrw_st = container_of(work, struct lrw_struct, rx_work);
	skb = lrw_st->rx_skb_list.next;
	skb_dequeue(&lrw_st->rx_skb_list);

	/* Check and parse the RX frame */
	ss = lrw_rx_skb_2_session(lrw_st, skb);
	if (!ss)
		goto lrw_rx_work_not_new_frame;

	lrw_parse_frame(ss, skb);

	/* Check the TX frame is acked or not */
	if (ss->tx_should_ack) {
		ss->rx_skb = NULL;
		goto lrw_rx_work_not_new_frame;
	}

	/* The TX frame is acked or no need to be acked */
	del_timer(&ss->timer);
	lrw_st->rx_should_ack = (ss->rx_fhdr.mtype & 0xC0) == 0x40;

	mutex_lock(&lrw_st->ss_list_lock);
	lrw_st->fcnt_down = ss->rx_fhdr.fcnt;
	lrw_st->_cur_ss = NULL;

	if (ss->rx_skb->len == 0) {
		/* Remove the session if it is no RX frame payload */
		lrw_del_ss(ss);
		mutex_unlock(&lrw_st->ss_list_lock);
	}
	else {
		mutex_unlock(&lrw_st->ss_list_lock);

		spin_lock_bh(&ss->state_lock);
		ss->state = LRW_RXRECEIVED_SS;
		spin_unlock_bh(&ss->state_lock);
	}

	return;

lrw_rx_work_not_new_frame:
	/* Drop the RX frame if checked failed */
	kfree_skb(skb);
}

int
lrw_check_mic(struct crypto_shash *tfm, struct sk_buff *skb)
{
	u8 *buf;
	size_t len;
	u8 *devaddr;
	u16 fcnt;
	__le16 *p_fcnt;
	u8 cks[4];
	u8 *mic;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	buf = skb->data;
	len = skb->len - 4;
	devaddr = buf + 1;
	p_fcnt = (__le16 *)(buf + 6);
	fcnt = le16_to_cpu(*p_fcnt);
	mic = skb->data + len;

	lrw_calc_mic(tfm, LRW_DOWNLINK, devaddr, fcnt, buf, len, cks);

	return (!memcmp(cks, mic, 4));
}

void
lora_rx_irqsave(struct lora_hw *hw, struct sk_buff *skb)
{
	struct lrw_struct *lrw_st;
	u8 mtype;
	bool is_new_frame;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	lrw_st = container_of(hw, struct lrw_struct, hw);
	mtype = skb->data[0] >> 5;
	is_new_frame = 0;

	/* Check the frame is downlink frame */
	if (((mtype == LRW_UNCONFIRMED_DATA_DOWN)
	      || (mtype == LRW_CONFIRMED_DATA_DOWN))
	    // && activated
	    && (memcmp(lrw_st->devaddr, skb->data + LRW_MHDR_LEN, 4) != 0)
	    && lrw_check_mic(lrw_st->nwks_shash_tfm, skb))
		is_new_frame = true;
	//else if ((need to be auto activated) && (mtype == LRW_JOIN_ACCEPT))
	//	is_new_frame = true;

	if (is_new_frame) {
		skb_queue_tail(&lrw_st->rx_skb_list, skb);
		schedule_work(&lrw_st->rx_work);
	}
	else {
		kfree_skb(skb);
	}
}
EXPORT_SYMBOL(lora_rx_irqsave);

static void
lrw_rexmit(struct timer_list *timer)
{
	struct lrw_session *ss = container_of(timer, struct lrw_session, timer);
	struct lrw_struct *lrw_st = ss->lrw_st;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	lrw_xmit((unsigned long) lrw_st);
}

static void
rx_timeout_work(struct work_struct *work)
{
	struct lrw_struct *lrw_st;
	struct lrw_session *ss;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	ss = container_of(work, struct lrw_session, timeout_work);
	lrw_st = ss->lrw_st;

	mutex_lock(&lrw_st->ss_list_lock);
	lrw_st->_cur_ss = NULL;
	lrw_del_ss(ss);
	mutex_unlock(&lrw_st->ss_list_lock);
}

static void
rx2_timeout_isr(struct timer_list *timer)
{
	struct lrw_session *ss = container_of(timer, struct lrw_session, timer);

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	/* Check TX is acked or not */
	if (!ss->tx_should_ack) {
		spin_lock_bh(&ss->state_lock);
		if (ss->state != LRW_RXRECEIVED_SS)
			ss->state = LRW_RXTIMEOUT_SS;
		spin_unlock_bh(&ss->state_lock);

		if (ss->state == LRW_RXTIMEOUT_SS)
			goto rx2_timeout_isr_no_retry_rx_frame;
		else
			return;
	}

	/* Check the session need to be retransmitted or not */
	if (ss->retry > 0) {
		ss->state = LRW_RETRANSMIT_SS;
		ss->retry--;

		/* Start timer for ack timeout and retransmit */
		ss->timer.function = lrw_rexmit;
		ss->timer.expires = jiffies_64 + ss->ack_timeout * HZ;
		add_timer(&ss->timer);
	}
	else {
		/* Retry failed */
rx2_timeout_isr_no_retry_rx_frame:
		schedule_work(&ss->timeout_work);
	}
}

static void
rx2_delay_isr(struct timer_list *timer)
{
	struct lrw_session *ss = container_of(timer, struct lrw_session, timer);
	struct lrw_struct *lrw_st = ss->lrw_st;
	unsigned long delay;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	/* Start timer for RX2 window */
	ss->timer.function = rx2_timeout_isr;
	delay = jiffies_64 + (ss->rx2_window + 20) * HZ / 1000 + HZ;
	ss->timer.expires = delay;
	add_timer(&ss->timer);

	/* Start LoRa hardware to RX2 window */
	ss->state = LRW_RX2_SS;
	lrw_st->ops->start_rx_window(&lrw_st->hw, ss->rx2_window + 20);
}

static void
rx1_delay_isr(struct timer_list *timer)
{
	struct lrw_session *ss = container_of(timer, struct lrw_session, timer);
	struct lrw_struct *lrw_st = ss->lrw_st;
	unsigned long delay;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	/* Start timer for RX_Delay2 - RX_Delay2 */
	ss->timer.function = rx2_delay_isr;
	delay = jiffies_64 + (ss->rx_delay2 - ss->rx_delay1) * HZ - 20 * HZ / 1000;
	ss->timer.expires = delay;
	add_timer(&ss->timer);

	/* Start LoRa hardware to RX1 window */
	ss->state = LRW_RX1_SS;
	lrw_st->ops->start_rx_window(&lrw_st->hw, ss->rx1_window + 20);
}

void
lrw_sent_tx_work(struct lrw_struct *lrw_st, struct sk_buff *skb)
{
	struct lrw_session *ss = lrw_st->_cur_ss;
	struct net_device *ndev;
	unsigned long delay;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	ss->state = LRW_XMITTED;

	/* Start session timer for RX_Delay1 */
	timer_setup(&ss->timer, rx1_delay_isr, 0);
	delay = jiffies_64 + ss->rx_delay1 * HZ - 20 * HZ / 1000;
	ss->timer.expires = delay;
	add_timer(&ss->timer);

	ndev = skb->dev;
	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;
	dev_consume_skb_any(skb);

	/* Set LoRa hardware to IDLE state */
	lrw_st->ops->set_state(&lrw_st->hw, LORA_STATE_IDLE);
}

void
lora_xmit_complete(struct lora_hw *hw, struct sk_buff *skb)
{
	struct lrw_struct *lrw_st;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	lrw_st = container_of(hw, struct lrw_struct, hw);
	lrw_sent_tx_work(lrw_st, skb);
}
EXPORT_SYMBOL(lora_xmit_complete);

int
lrw_get_devaddr(struct lora_hw *hw, u8 *devaddr)
{
	struct lrw_struct *lrw_st;

	lrw_st = container_of(hw, struct lrw_struct, hw);
	memcpy(devaddr, lrw_st->devaddr, LRW_DEVADDR_LEN);

	return 0;
}
EXPORT_SYMBOL(lrw_get_devaddr);
