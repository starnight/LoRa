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
#include <linux/interrupt.h>

#include "lora.h"
#include "lorawan.h"

static struct class *lrw_sys_class;
static int lrw_major;
#define	LORA_HW_AMOUNT		(sizeof(int) * 8)

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static DECLARE_BITMAP(minors, LORA_HW_AMOUNT);
static DEFINE_MUTEX(minors_lock);

struct lora_hw *
lora_alloc_hw(size_t priv_data_len, struct lora_operations *ops)
{
	struct lrw_struct *lrw_st;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	if (WARN_ON(!ops || !ops->start || !ops->stop || !ops->xmit_async ||
		    !ops->set_txpower || !ops->set_frq || !ops->set_bw ||
		    !ops->set_mod || !ops->set_sf || !ops->start_rx_window ||
		    !ops->set_state))
		return NULL;

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

	lrw_st->state = LORA_STOP;
	lrw_st->ops = ops;
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

/**
 * lrw_add_hw - Add a LoRaWAN compatible hardware into the device list
 * @lrw_st:	the LoRa device going to be added
 *
 * Return:	0 / other number for success / failed
 */
int
lrw_add_hw(struct lrw_struct *lrw_st)
{
	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	INIT_LIST_HEAD(&(lrw_st->device_entry));

	mutex_lock(&device_list_lock);
	list_add(&lrw_st->device_entry, &device_list);
	mutex_unlock(&device_list_lock);

	lrw_st->fcnt_up = 0;
	lrw_st->fcnt_down = 0;
	lrw_st->_cur_ss = NULL;

	mutex_init(&lrw_st->ss_list_lock);
	INIT_LIST_HEAD(&lrw_st->ss_list);

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
	int ret = 0;

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

	pr_debug("%s: write a new skb\n", LORAWAN_MODULE_NAME);
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
	void *pval;
	struct lrw_struct *lrw_st;

	pr_debug("%s: ioctl file (cmd=0x%X)\n", LORAWAN_MODULE_NAME, cmd);

	pval = (void __user *)arg;
	lrw_st = filp->private_data;

	/* I/O control by each command */
	switch (cmd) {
	/* Set & read the state of the LoRa device */
	case LRW_SET_STATE:
		ret = lrw_set_hw_state(lrw_st, (u8 *)pval);
		break;
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
	unsigned int minor;
	int status;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);
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
	cdev_init(&lrw_st->lrw_cdev, &lrw_fops);
	lrw_st->lrw_cdev.owner = THIS_MODULE;
	lrw_st->devt = MKDEV(lrw_major, minor);
	status = cdev_add(&lrw_st->lrw_cdev, lrw_st->devt, LORA_HW_AMOUNT);
	if (status)
		goto lrw_register_hw_end;
	lrw_st->dev = device_create(lrw_sys_class, NULL, lrw_st->devt, lrw_st,
				    "lora%d", minor);
	status = PTR_ERR_OR_ZERO(lrw_st->dev);

lrw_register_hw_end:
	if (status == 0) {
		lrw_add_hw(lrw_st);
		//lora_start_hw(lrw_st);
	}
	else {
		mutex_lock(&minors_lock);
		clear_bit(minor, minors);
		mutex_unlock(&minors_lock);
	}

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
	if (lrw_st->state != LORA_STOP)
		lora_stop_hw(lrw_st);
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
