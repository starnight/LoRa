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
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/interrupt.h>

#include "lora.h"
#include "lorawan.h"

#define	PHY_NAME		"lora"

static struct class *lrw_sys_class;

static void
lora_if_setup(struct net_device *ndev)
{
	ndev->hard_header_len = LRW_MHDR_LEN + LRW_FHDR_MAX_LEN + LRW_FPORT_LEN;
	ndev->needed_tailroom = LRW_MIC_LEN;
	// TODO: M is a dynamic value defined by Regional Parameters
	//ndev->mtu = M - ndev->hard_header_len;
	ndev->mtu = 20;
}

/**
 * lora_alloc_hw - Allocate a memory space for the LoRa device
 * @priv_data_len:	the private data size
 * @lora_operations:	the implemented operations of the LoRa device
 *
 * Return:		address of the LoRa device or NULL for failed
 */
struct lora_hw *
lora_alloc_hw(size_t priv_data_len, struct lora_operations *ops)
{
	struct net_device *ndev;
	struct lrw_struct *lrw_st;
	int ret;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	if (WARN_ON(!ops || !ops->start || !ops->stop || !ops->xmit_async ||
		    !ops->set_txpower || !ops->set_frq || !ops->set_bw ||
		    !ops->set_mod || !ops->set_sf || !ops->start_rx_window ||
		    !ops->set_state))
		return NULL;

	/* In memory it'll be like this:
	 *
	 * +-----------------------+
	 * | struct net_device     |
	 * +-----------------------+
	 * | struct lrw_struct     |
	 * +-----------------------+
	 * | driver's private data |
	 * +-----------------------+
	 */
	ndev = alloc_netdev(sizeof(struct lrw_struct) + priv_data_len,
			    PHY_NAME"%d", NET_NAME_ENUM, lora_if_setup);
	if (!ndev)
		return ERR_PTR(-ENOMEM);
	ret = dev_alloc_name(ndev, ndev->name);
	if (ret < 0)
		goto lora_alloc_hw_err;

	lrw_st = (struct lrw_struct *)netdev_priv(ndev);
	lrw_st->ndev = ndev;

	lrw_st->state = LORA_STOP;
	lrw_st->ops = ops;
	lrw_st->hw.priv = (void *) lrw_st + sizeof(struct lrw_struct);

	SET_NETDEV_DEV(ndev, &lrw_st->dev);
	dev_net_set(ndev, &lrw_st->net);
	ndev->flags |= IFF_NOARP;
	ndev->features |= NETIF_F_HW_CSUM;

	return &lrw_st->hw;

lora_alloc_hw_err:
	free_netdev(ndev);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(lora_alloc_hw);

/**
 * lora_free_hw - Free the LoRa device's memory resource
 * @hw:		the LoRa device going to be freed
 */
void
lora_free_hw(struct lora_hw *hw)
{
	struct lrw_struct *lrw_st;

	lrw_st = container_of(hw, struct lrw_struct, hw);
	free_netdev(lrw_st->ndev);
}
EXPORT_SYMBOL(lora_free_hw);

/**
 * lrw_add_hw - Add a LoRaWAN hardware as a network device
 * @lrw_st:	the LoRa device going to be added
 *
 * Return:	0 / other number for success / failed
 */
int

lrw_add_hw(struct lrw_struct *lrw_st)
{
	struct net_device *ndev = lrw_st->ndev;
	int ret;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	lrw_st->fcnt_up = 0;
	lrw_st->fcnt_down = 0;
	lrw_st->_cur_ss = NULL;

	mutex_init(&lrw_st->ss_list_lock);
	INIT_LIST_HEAD(&lrw_st->ss_list);

	tasklet_init(&lrw_st->xmit_task, lrw_xmit, (unsigned long) lrw_st);
	INIT_WORK(&lrw_st->rx_work, lrw_rx_work);

	ret = register_netdev(ndev);

	return ret;
}

/**
 * lrw_remove_hw - Remove a LoRaWAN hardware from a network device
 * @lrw_st:	the LoRa device going to be removed
 */
void
lrw_remove_hw(struct lrw_struct *lrw_st)
{
	unregister_netdev(lrw_st->ndev);
	tasklet_kill(&lrw_st->xmit_task);
}

int
lrw_set_hw_state(struct lrw_struct *lrw_st, void __user *arg)
{
	int ret = 0;
	u8 state;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);
	copy_from_user(&state, arg, 1);
	switch (state) {
	case LORA_START:
		if (lrw_st->state == LORA_STOP)
			lora_start_hw(lrw_st);
		break;
	case LORA_STOP:
		if (lrw_st->state != LORA_STOP)
			lora_stop_hw(lrw_st);
		break;
	default:
		ret = -ENOTSUPP;
	}

	return ret;
}

bool
ready2write(struct lrw_struct *lrw_st)
{
	bool status = false;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);
	if ((!lrw_st->_cur_ss) && (lrw_st->state != LORA_STOP))
		status = true;

	return status;
}

bool
ready2read(struct lrw_struct *lrw_st)
{
	bool status = false;
	struct lrw_session *ss;

	if (!list_empty(&lrw_st->ss_list) && (lrw_st->state != LORA_STOP)) {
		ss = list_first_entry(&lrw_st->ss_list,
				      struct lrw_session,
				      entry);
		if (ss->state == LRW_RXRECEIVED_SS)
			status = true;
	}

	return status;
}

int
lora_set_key(struct lora_hw *hw, u8 type, u8 *key, size_t l)
{
	int ret;
	struct lrw_struct *lrw_st;

	lrw_st = container_of(hw, struct lrw_struct, hw);

	ret = 0;
	switch (type) {
	case LORA_APPKEY:
		memcpy(lrw_st->appkey, key, LORA_KEY_LEN);
		break;
	case LORA_NWKSKEY:
		memcpy(lrw_st->nwkskey, key, LORA_KEY_LEN);
		break;
	case LORA_APPSKEY:
		memcpy(lrw_st->appskey, key, LORA_KEY_LEN);
		break;
	default:
		ret = -1;
	}

	return ret;
}

static int
lrw_if_up(struct net_device *ndev)
{
	struct lrw_struct *lrw_st = NETDEV_2_LRW(ndev);
	int ret = 0;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	if (lrw_st->state == LORA_STOP) {
		ret = lora_start_hw(lrw_st);
		netif_start_queue(ndev);
	}
	else if (lrw_st->state != LORA_START)
		ret = -EBUSY;

	return ret;
}

static int
lrw_if_down(struct net_device *ndev)
{
	struct lrw_struct *lrw_st = NETDEV_2_LRW(ndev);

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	if (lrw_st->state != LORA_STOP) {
		netif_stop_queue(ndev);
		lora_stop_hw(lrw_st);
	}

	return 0;
}

netdev_tx_t
lrw_if_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct lrw_struct *lrw_st = NETDEV_2_LRW(ndev);
	struct lrw_session *ss;
	netdev_tx_t ret = NETDEV_TX_OK;

	pr_debug("%s: xmit skb (size=%u)\n", LORAWAN_MODULE_NAME, skb->len);

	ss = lrw_alloc_ss(lrw_st);
	if (!ss)
		return NETDEV_TX_BUSY;

	mutex_lock(&lrw_st->ss_list_lock);
	if (ready2write(lrw_st)) {
		list_add_tail(&ss->entry, &lrw_st->ss_list);
		lrw_st->_cur_ss = ss;
		lrw_st->fcnt_up += 1;
		ss->fcnt_up = lrw_st->fcnt_up;
		ss->fcnt_down = lrw_st->fcnt_down;
	}
	else
		ret = NETDEV_TX_BUSY;
	mutex_unlock(&lrw_st->ss_list_lock);

	if (ret == NETDEV_TX_OK) {
		pr_debug("%s: write a new skb\n", LORAWAN_MODULE_NAME);
		ss->state = LRW_INIT_SS;
		ss->tx_skb = skb;
		lrw_prepare_tx_frame(ss);
		tasklet_schedule(&lrw_st->xmit_task);
	}
	else
		lrw_free_ss(ss);

	return ret;
}

static int
lrw_if_ioctl(struct net_device *ndev, struct ifreq *ifr, int cmd)
{
	struct lrw_struct *lrw_st = NETDEV_2_LRW(ndev);
	long ret;

	pr_debug("%s: ioctl file (cmd=0x%X)\n", LORAWAN_MODULE_NAME, cmd);

	/* I/O control by each command */
	switch (cmd) {
	/* Set & read the state of the LoRa device */
//	case LRW_SET_STATE:
//		ret = lrw_set_hw_state(lrw_st, (u8 *)pval);
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
		ret = -ENOTSUPP;
	}

	return ret;
}

static int
lrw_if_set_mac(struct net_device *ndev, void *p)
{
	struct lrw_struct *lrw_st = NETDEV_2_LRW(ndev);

	return 0;
}

static const struct net_device_ops lrw_if_ops = {
	.ndo_open = lrw_if_up,
	.ndo_stop = lrw_if_down,
	.ndo_start_xmit = lrw_if_start_xmit,
	.ndo_do_ioctl = lrw_if_ioctl,
	.ndo_set_mac_address = lrw_if_set_mac,
};

/**
 * lora_register_hw - Register there is a kind of LoRa device
 * @hw:		LoRa device going to be registered
 *
 * Return:	0 / negative number for success / error number
 */
int
lora_register_hw(struct lora_hw *hw)
{
	struct lrw_struct *lrw_st = container_of(hw, struct lrw_struct, hw);
	int ret;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	/* Add a LoRa device node as a network device */
	ret = lrw_add_hw(lrw_st);
	if (ret < 0)
		goto lora_register_hw_end;

	device_initialize(&lrw_st->dev);
	dev_set_name(&lrw_st->dev, netdev_name(lrw_st->ndev));
	lrw_st->dev.class = lrw_sys_class;
	lrw_st->dev.platform_data = lrw_st;

lora_register_hw_end:
	return ret;
}
EXPORT_SYMBOL(lora_register_hw);

/**
 * lora_unregister_hw - Unregister the LoRa driver
 * @hw:		LoRa device going to be unregistered
 */
void
lora_unregister_hw(struct lora_hw *hw)
{
	struct lrw_struct *lrw_st = container_of(hw, struct lrw_struct, hw);

	pr_info("%s: unregister %s\n",
		LORAWAN_MODULE_NAME, dev_name(&lrw_st->dev));

	/* Stop and remove the LoRaWAM hardware from system */
	if (lrw_st->state != LORA_STOP)
		lora_stop_hw(lrw_st);
	lrw_remove_hw(lrw_st);

	return;
}
EXPORT_SYMBOL(lora_unregister_hw);

static int __init
lrw_init(void)
{
	int err;

	pr_info("%s: module inserted\n", LORAWAN_MODULE_NAME);

	/* Create device class */
	lrw_sys_class = class_create(THIS_MODULE, LORAWAN_MODULE_NAME);
	if (IS_ERR(lrw_sys_class)) {
		pr_err("%s: Failed to create a class of LoRaWAN\n",
		       LORAWAN_MODULE_NAME);
		err = PTR_ERR(lrw_sys_class);
	}
	else {
		pr_debug("%s: class created\n", LORAWAN_MODULE_NAME);
		err = 0;
	}

	return err;
}

static void __exit
lrw_exit(void)
{
	/* Delete device class */
	class_destroy(lrw_sys_class);
	pr_info("%s: module removed\n", LORAWAN_MODULE_NAME);

	return;
}

module_init(lrw_init);
module_exit(lrw_exit);

MODULE_AUTHOR("Jian-Hong Pan, <starnight@g.ncu.edu.tw>");
MODULE_DESCRIPTION("LoRaWAN kernel module");
MODULE_LICENSE("Dual BSD/GPL");
