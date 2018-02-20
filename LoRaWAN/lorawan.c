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
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/interrupt.h>

#include "lora.h"
#include "lorawan.h"
#include "lrwsec.h"

#define	LORAWAN_MODULE_NAME	"lorawan"

struct lora_hw *
lora_alloc_hw(size_t priv_data_len, struct lora_operations *lr_ops)
{
	struct lrw_struct *lrw_st;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);
	/* In memory it'll be like this:
	 *
	 * +-----------------------+
	 * | struct lrw_struct     |
	 * +-----------------------+
	 * | driver's private data |
	 * +-----------------------+
	 */

	lrw_st = kzalloc(sizeof(struct lrw_struct) + priv_data_len, GFP_KERNEL);
	if (!lrw_st)
		return NULL;

	lrw_st->ops = lr_ops;
	lrw_st->hw.priv = (void *) lrw_st + sizeof(struct lrw_struct);

	return &lrw_st->hw;
}
EXPORT_SYMBOL(lora_alloc_hw);

void
lora_free_hw(struct lora_hw *hw)
{
	struct lrw_struct *lrw_st;

	lrw_st = container_of(hw, struct lrw_struct, hw);
	kfree(lrw_st);
}
EXPORT_SYMBOL(lora_free_hw);

static void rx_timeout_work(struct work_struct *work);

struct lrw_session *
lrw_alloc_ss(struct lrw_struct *lrw_st)
{
	struct lrw_session *ss;

	ss = kzalloc(sizeof(struct lrw_session), GFP_KERNEL);
	if (!ss)
		goto lrw_alloc_ss_end;
	
	ss->lrw_st = lrw_st;
	memcpy(ss->devaddr, lrw_st->devaddr, LRW_DEVADDR_LEN);
	ss->appkey = lrw_st->appkey;
	ss->nwkskey = lrw_st->nwkskey;
	ss->appskey = lrw_st->appskey;

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

bool
ready2write(struct lrw_struct *lrw_st)
{
	bool status = false;

	if (!lrw_st->_cur_ss)
		status = true;

	return status;
}

bool
ready2read(struct lrw_struct *lrw_st)
{
	bool status = false;
	struct lrw_session *ss;

	if (!list_empty(&lrw_st->ss_list)) {
		ss = list_first_entry(&lrw_st->ss_list,
				      struct lrw_session,
				      entry);
		if (ss->state == LRW_RXRECEIVED_SS)
			status = true;
	}

	return status;
}

void
lrw_prepare_tx_frame(struct lrw_session *ss)
{
	struct lrw_struct *lrw_st = ss->lrw_st;
	struct sk_buff *skb = ss->tx_skb;
	u8 mhdr, fctrl, fport;
	u8 mic[4];

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
		memcpy(skb_push(skb, 1), &fport, 1);
	}

	/* Push FCnt_Up */
	memcpy(skb_push(skb, 2), &ss->fcnt_up, 2);

	/* Push FCtrl */
	memcpy(skb_push(skb, 1), &fctrl, 1);

	/* Push DevAddr */
	memcpy(skb_push(skb, LRW_DEVADDR_LEN), ss->devaddr, LRW_DEVADDR_LEN);

	/* Push MHDR */
	memcpy(skb_push(skb, 1), &mhdr, 1);

	/* Put MIC */
	lrw_calc_mic(lrw_st->nwks_shash_tfm, LRW_UPLINK,
		     ss->devaddr, ss->fcnt_up, skb->data, skb->len, mic);
	memcpy(skb_put(skb, 4), mic, 4);
}

static void
lrw_xmit(unsigned long data)
{
	struct lrw_struct *lrw_st = (struct lrw_struct *) data;
	struct lrw_session *ss = lrw_st->_cur_ss;

	ss->state = LRW_XMITTING_SS;
	lrw_st->ops->xmit_async(&lrw_st->hw, ss->tx_skb);
}

void
lrw_parse_frame(struct lrw_session *ss, struct sk_buff *skb)
{
	struct lrw_fhdr *fhdr = &ss->rx_fhdr;
	__le16 *p_fcnt;

	/* Get message type */
	fhdr->mtype = skb->data[0];
	skb_pull(skb, 1);

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
	skb_trim(skb, skb->len - 4);
}

struct lrw_session *
lrw_rx_skb_2_session(struct lrw_struct *lrw_st, struct sk_buff *rx_skb)
{
	struct lrw_session *ss;
	u16 fcnt;
	__le16 *p_fcnt;

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

static void
lrw_rx_work(struct work_struct *work)
{
	struct lrw_struct *lrw_st;
	struct lrw_session *ss;
	struct sk_buff *skb;

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

	lrw_st = container_of(hw, struct lrw_struct, hw);
	mtype = skb->data[0] >> 5;
	is_new_frame = 0;

	/* Check the frame is downlink frame */
	if (((mtype == LRW_UNCONFIRMED_DATA_DOWN)
	      || (mtype == LRW_CONFIRMED_DATA_DOWN))
	    // && activated
	    && (memcmp(lrw_st->devaddr, skb->data + 1, 4) != 0)
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

	lrw_xmit((unsigned long) lrw_st);
}

static void
rx_timeout_work(struct work_struct *work)
{
	struct lrw_struct *lrw_st;
	struct lrw_session *ss;

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

	/* Start timer for RX2 window */
	ss->timer.function = rx2_timeout_isr;
	delay = jiffies_64 + (ss->rx2_window + 20) * HZ / 1000 + HZ;
	ss->timer.expires = delay;
	add_timer(&ss->timer);

	/* Start LoRa hardware to RX2 window */
	ss->state = LRW_RX2_SS;
	lrw_st->ops->start_rx2_window(&lrw_st->hw, ss->rx2_window + 20);
}

static void
rx1_delay_isr(struct timer_list *timer)
{
	struct lrw_session *ss = container_of(timer, struct lrw_session, timer);
	struct lrw_struct *lrw_st = ss->lrw_st;
	unsigned long delay;

	/* Start timer for RX_Delay2 - RX_Delay2 */
	ss->timer.function = rx2_delay_isr;
	delay = jiffies_64 + (ss->rx_delay2 - ss->rx_delay1) * HZ - 20 * HZ / 1000;
	ss->timer.expires = delay;
	add_timer(&ss->timer);

	/* Start LoRa hardware to RX1 window */
	ss->state = LRW_RX1_SS;
	lrw_st->ops->start_rx1_window(&lrw_st->hw, ss->rx1_window + 20);
}

static void
lrw_sent_tx_work(struct lrw_struct *lrw_st, struct sk_buff *skb)
{
	struct lrw_session *ss;
	unsigned long delay;

	/* Find the session in ss_list_entry with matched skb */
	ss = lrw_st->_cur_ss;	

	ss->state = LRW_XMITTED;

	/* Start session timer for RX_Delay1 */
	timer_setup(&ss->timer, rx1_delay_isr, 0);
	delay = jiffies_64 + ss->rx_delay1 * HZ - 20 * HZ / 1000;
	ss->timer.expires = delay;
	add_timer(&ss->timer);

	/* Set LoRa hardware to IDLE state */
	lrw_st->ops->set_state(&lrw_st->hw, LORA_STATE_IDLE);
}

void
lora_xmit_complete(struct lora_hw *hw, struct sk_buff *skb)
{
	struct lrw_struct *lrw_st;

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

/* ---------------------- Character device driver part ---------------------- */

static struct class *lrw_sys_class;
static int lrw_major;
#define	LORA_HW_AMOUNT		(sizeof(int) * 8)

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static DECLARE_BITMAP(minors, LORA_HW_AMOUNT);
static DEFINE_MUTEX(minors_lock);

/**
 * lrw_add_hw - Add a LoRaWAN compatible hardware into the device list
 * @lrw_st:	the LoRa device going to be added
 *
 * Return:	0 / other number for success / failed
 */
int
lrw_add_hw(struct lrw_struct *lrw_st)
{
	INIT_LIST_HEAD(&(lrw_st->device_entry));

	mutex_lock(&device_list_lock);
	list_add(&lrw_st->device_entry, &device_list);
	mutex_unlock(&device_list_lock);

	lrw_st->fcnt_up = 0;
	lrw_st->fcnt_down = 0;
	lrw_st->_cur_ss = NULL;

	tasklet_init(&lrw_st->xmit_task, lrw_xmit, (unsigned long) lrw_st);
	INIT_WORK(&lrw_st->rx_work, lrw_rx_work);

	return 0;
}

/**
 * lrw_remove_hw - Remove a LoRaWAN compatible hardware from the device list
 * @lrw_st:	the LoRa device going to be removed
 *
 * Return:	0 / other number for success / failed
 */
int
lrw_remove_hw(struct lrw_struct *lrw_st)
{
	mutex_lock(&device_list_lock);
	list_del(&(lrw_st->device_entry));
	mutex_unlock(&device_list_lock);

	tasklet_kill(&lrw_st->xmit_task);

	return 0;
}

static int
file_open(struct inode *inode, struct file *filp)
{
	struct lrw_struct *lrw_st;
	int status = -ENXIO;

	pr_debug("%s: open file\n", LORAWAN_MODULE_NAME);

	mutex_lock(&device_list_lock);
	/* Find the lora data in device_entry with matched dev_t in inode */
	list_for_each_entry(lrw_st, &device_list, device_entry) {
		if (lrw_st->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status) {
		mutex_unlock(&device_list_lock);
		pr_debug("%s: nothing for minor %d\n",
			 LORAWAN_MODULE_NAME, iminor(inode));

		return status;
	}

	init_waitqueue_head(&(lrw_st->waitqueue));
	/* Map the data location to the file data pointer */
	filp->private_data = lrw_st;
	mutex_unlock(&device_list_lock);

	/* This a character device, so it is not seekable */
	nonseekable_open(inode, filp);

	return 0;
}

static int
file_close(struct inode *inode, struct file *filp)
{
	struct lrw_struct *lrw_st;

	pr_debug("%s: close file\n", LORAWAN_MODULE_NAME);

	lrw_st = filp->private_data;

	mutex_lock(&device_list_lock);
	filp->private_data = NULL;
	mutex_unlock(&device_list_lock);

	return 0;
}

static ssize_t
file_read(struct file *filp, char __user *buf, size_t size, loff_t *pos)
{
	struct lrw_struct *lrw_st;
	struct lrw_session *ss;
	struct sk_buff *skb;
	size_t len;
	ssize_t ret;

	pr_debug("%s: read file (size=%zu)\n", LORAWAN_MODULE_NAME, size);

	lrw_st = filp->private_data;

	mutex_lock(&lrw_st->ss_list_lock);
	if (ready2read(lrw_st)) {
		ss = list_first_entry(&lrw_st->ss_list,
				      struct lrw_session,
				      entry);
		skb = ss->rx_skb;
		len = (size <= skb->len) ? size : skb->len;
		if(!copy_to_user(buf, skb->data, len)) {
			ret = len;
			skb_pull(skb, len);
			if (!skb->len)
				lrw_del_ss(ss);
		}
		else {
			ret = -EFAULT;
		}
	}
	else {
		ret = -EBUSY;
	}
	mutex_unlock(&lrw_st->ss_list_lock);

	return ret;
}

static ssize_t
file_write(struct file *filp, const char __user *buf, size_t size, loff_t *pos)
{
	struct lrw_struct *lrw_st;
	struct lrw_session *ss;
	struct sk_buff *tx_skb;
	unsigned long rem;
	int ret;

	pr_debug("%s: write file (size=%zu)\n", LORAWAN_MODULE_NAME, size);

	lrw_st = filp->private_data;
	ss = NULL;

	mutex_lock(&lrw_st->ss_list_lock);
	if (ready2write(lrw_st)) {
		ss = lrw_alloc_ss(lrw_st);
		if (ss != NULL) {
			list_add_tail(&ss->entry, &lrw_st->ss_list);
			lrw_st->_cur_ss = ss;
			lrw_st->fcnt_up += 1;
			ss->fcnt_up = lrw_st->fcnt_up;
			ss->fcnt_down = lrw_st->fcnt_down;
		}
		else {
			ret = -ENOMEM;
		}
	}
	else {
		ret = -EBUSY;
	}
	mutex_unlock(&lrw_st->ss_list_lock);

	if (ss != NULL) {
		tx_skb = dev_alloc_skb(1 + 7 + 16 + size + 4);
		if (tx_skb != NULL) {
			ss->state = LRW_INIT_SS;
			ss->tx_skb = tx_skb;
			skb_reserve(tx_skb, 1 + 7 + 16);
			rem = copy_from_user(skb_put(tx_skb, size), buf, size);
			ret = size - rem;
			lrw_prepare_tx_frame(ss);
			tasklet_schedule(&lrw_st->xmit_task);
		}
		else {
			ret = -ENOMEM;
		}
	}

	return ret;
}

static long
file_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;
	int *pval;
	struct lrw_struct *lrw_st;

	pr_debug("%s: ioctl file (cmd=0x%X)\n", LORAWAN_MODULE_NAME, cmd);

	ret = -ENOTTY;
	pval = (void __user *)arg;
	lrw_st = filp->private_data;

	/* I/O control by each command */
	switch (cmd) {
//	/* Set & read the state of the LoRa device */
//	case LRW_SET_STATE:
//		if (lrw_st->ops->setState != NULL)
//			ret = lrw_st->ops->setState(lrw_st, pval);
//		break;
//	case LRW_GET_STATE:
//		if (lrw_st->ops->getState != NULL)
//			ret = lrw_st->ops->getState(lrw_st, pval);
//		break;
//	/* Set & get the carrier frequency */
//	case LRW_SET_FREQUENCY:
//		if (lrw_st->ops->setFreq != NULL)
//			ret = lrw_st->ops->setFreq(lrw_st, pval);
//		break;
//	case LRW_GET_FREQUENCY:
//		if (lrw_st->ops->getFreq != NULL)
//			ret = lrw_st->ops->getFreq(lrw_st, pval);
//		break;
//	/* Set & get the PA power */
//	case LRW_SET_POWER:
//		if (lrw_st->ops->setPower != NULL)
//			ret = lrw_st->ops->setPower(lrw_st, pval);
//		break;
//	case LRW_GET_POWER:
//		if (lrw_st->ops->getPower != NULL)
//			ret = lrw_st->ops->getPower(lrw_st, pval);
//		break;
//	/* Set & get the LNA gain */
//	case LRW_SET_LNA:
//		if (lrw_st->ops->setLNA != NULL)
//			ret = lrw_st->ops->setLNA(lrw_st, pval);
//		break;
//	case LRW_GET_LNA:
//		if (lrw_st->ops->getLNA != NULL)
//			ret = lrw_st->ops->getLNA(lrw_st, pval);
//		break;
//	/* Set the LNA be auto gain control or manual */
//	case LRW_SET_LNAAGC:
//		if (lrw_st->ops->setLNAAGC != NULL)
//			ret = lrw_st->ops->setLNAAGC(lrw_st, pval);
//		break;
//	/* Set & get the RF spreading factor */
//	case LRW_SET_SPRFACTOR:
//		if (lrw_st->ops->setSPRFactor != NULL)
//			ret = lrw_st->ops->setSPRFactor(lrw_st, pval);
//		break;
//	case LRW_GET_SPRFACTOR:
//		if (lrw_st->ops->getSPRFactor != NULL)
//			ret = lrw_st->ops->getSPRFactor(lrw_st, pval);
//		break;
//	/* Set & get the RF bandwith */
//	case LRW_SET_BANDWIDTH:
//		if (lrw_st->ops->setBW != NULL)
//			ret = lrw_st->ops->setBW(lrw_st, pval);
//		break;
//	case LRW_GET_BANDWIDTH:
//		if (lrw_st->ops->getBW != NULL)
//			ret = lrw_st->ops->getBW(lrw_st, pval);
//		break;
//	/* Get current RSSI */
//	case LRW_GET_RSSI:
//		if (lrw_st->ops->getRSSI != NULL)
//			ret = lrw_st->ops->getRSSI(lrw_st, pval);
//		break;
//	/* Get last packet's SNR */
//	case LRW_GET_SNR:
//		if (lrw_st->ops->getSNR != NULL)
//			ret = lrw_st->ops->getSNR(lrw_st, pval);
//		break;
	default:
		ret = -ENOTTY;
	}

	return ret;
}

static unsigned int
file_poll(struct file *filp, poll_table *wait)
{
	struct lrw_struct *lrw_st;
	unsigned int mask;

	pr_debug("%s: poll file\n", LORAWAN_MODULE_NAME);

	lrw_st = filp->private_data;

	/* Register the file into wait queue for multiplexing */
	poll_wait(filp, &lrw_st->waitqueue, wait);

	/* Check ready to write / read */
	mask = 0;
	mutex_lock(&lrw_st->ss_list_lock);
	if (ready2write(lrw_st))
		mask |= POLLOUT | POLLWRNORM;
	if (ready2read(lrw_st))
		mask |= POLLIN | POLLRDNORM;
	mutex_unlock(&lrw_st->ss_list_lock);

	return mask;
}

static struct file_operations lrw_fops = {
	.owner		= THIS_MODULE,
	.open 		= file_open,
	.release	= file_close,
	.read		= file_read,
	.write		= file_write,
	.unlocked_ioctl = file_ioctl,
	.poll		= file_poll,
	.llseek		= no_llseek,
};

/**
 * lora_register_hw - Register there is a kind of LoRa driver
 * @driver:	LoRa driver going to be registered
 *
 * Return:	0 / negative number for success / error number
 */
int
lora_register_hw(struct lora_hw *hw)
{
	struct lrw_struct *lrw_st;
	struct device *dev;
	unsigned int minor;
	int status = 0;

	lrw_st = container_of(hw, struct lrw_struct, hw);

	/* Check there is a space for new LoRa hardware */
	mutex_lock(&minors_lock);
	minor = find_first_zero_bit(minors, LORA_HW_AMOUNT);
	if (minor < LORA_HW_AMOUNT)
		set_bit(minor, minors);
	mutex_unlock(&minors_lock);
	if (minor >= LORA_HW_AMOUNT) {
		status = -ENODEV;
		goto lrw_register_hw_end;
	}

	/* Add a LoRa device node under /dev */
	cdev_init(&(lrw_st->lrw_cdev), &lrw_fops);
	lrw_st->lrw_cdev.owner = THIS_MODULE;
	lrw_st->devt = MKDEV(lrw_major, minor);
	lrw_st->dev = device_create(lrw_sys_class, NULL, lrw_st->devt, lrw_st,
				    "lora%d", minor);
	status = PTR_ERR_OR_ZERO(dev);
	if (status == 0)
		lrw_add_hw(lrw_st);
	else {
		mutex_lock(&minors_lock);
		clear_bit(minor, minors);
		mutex_unlock(&minors_lock);
	}

lrw_register_hw_end:
	return status;
}
EXPORT_SYMBOL(lora_register_hw);

/**
 * lora_unregister_hw - Unregister the LoRa driver
 * @driver:	LoRa driver going to be unregistered
 */
void
lora_unregister_hw(struct lora_hw *hw)
{
	struct lrw_struct *lrw_st;
	int minor;

	lrw_st = container_of(hw, struct lrw_struct, hw);
	minor = MINOR(lrw_st->devt);

	pr_info("%s: unregister lora%d\n", LORAWAN_MODULE_NAME, minor);

	/* Delete the character device driver from system */
	cdev_del(&(lrw_st->lrw_cdev));
	device_destroy(lrw_sys_class, lrw_st->devt);
	lrw_remove_hw(lrw_st);

	mutex_lock(&minors_lock);
	clear_bit(minor, minors);
	mutex_unlock(&minors_lock);

	return;
}
EXPORT_SYMBOL(lora_unregister_hw);

static int
lrw_init(void)
{
	dev_t devt;
	int alloc_ret = -1;
	int err;

	pr_info("%s: module inserted\n", LORAWAN_MODULE_NAME);

	/* Allocate a character device */
	alloc_ret = alloc_chrdev_region(&devt,
					0,
					LORA_HW_AMOUNT,
					LORAWAN_MODULE_NAME);
	if (alloc_ret) {
		pr_err("%s: Failed to allocate a character device\n",
		       LORAWAN_MODULE_NAME);
		err = alloc_ret;
		goto lrw_init_error;
	}

	lrw_major = MAJOR(devt);

	/* Create device class */
	lrw_sys_class = class_create(THIS_MODULE, LORAWAN_MODULE_NAME);
	if (IS_ERR(lrw_sys_class)) {
		pr_err("%s: Failed to create a class of LoRaWAN\n",
		       LORAWAN_MODULE_NAME);
		err = PTR_ERR(lrw_sys_class);
		goto lrw_init_error;
	}
	pr_debug("%s: class created\n", LORAWAN_MODULE_NAME);

	return 0;

lrw_init_error:
	/* Release the allocated character device */
	if (alloc_ret == 0)
		unregister_chrdev_region(devt, LORA_HW_AMOUNT);
	return err;
}

static void
lrw_exit(void)
{
	dev_t devt = MKDEV(lrw_major, 0);

	/* Delete device class */
	class_destroy(lrw_sys_class);
	/* Unregister the allocated character device */
	unregister_chrdev_region(devt, LORA_HW_AMOUNT);
	pr_info("%s: module removed\n", LORAWAN_MODULE_NAME);

	return;
}

module_init(lrw_init);
module_exit(lrw_exit);

MODULE_AUTHOR("Jian-Hong Pan, <starnight@g.ncu.edu.tw>");
MODULE_DESCRIPTION("LoRaWAN kernel module");
MODULE_LICENSE("Dual BSD/GPL");
