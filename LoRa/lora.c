/*-
 * Copyright (c) 2017 Jian-Hong, Pan <starnight@g.ncu.edu.tw>
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

#include "lora.h"

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

#ifndef LORA_BUFLEN
#define LORA_BUFLEN	127
#endif

static int
file_open(struct inode *inode, struct file *filp)
{
	struct lora_struct *lrdata;
	int status = -ENXIO;

	pr_debug("lora: open file\n");
	
	mutex_lock(&device_list_lock);
	/* Find the lora data in device_entry with matched dev_t in inode. */
	list_for_each_entry(lrdata, &device_list, device_entry) {
		if (lrdata->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status) {
		pr_debug("lora: nothing for minor %d\n", iminor(inode));
		goto err_find_dev;
	}

	/* Have the RX/TX memory buffer. */
	if (!(lrdata->rx_buf)) {
		lrdata->rx_buf = kzalloc(LORA_BUFLEN, GFP_KERNEL);
		if (!(lrdata->rx_buf)) {
			pr_err("lora: no more memory\n");
			status = -ENOMEM;
			goto err_find_dev;
		}
	}
	if (!(lrdata->tx_buf)) {
		lrdata->tx_buf = kzalloc(LORA_BUFLEN, GFP_KERNEL);
		if (!(lrdata->tx_buf)) {
			pr_err("lora: no more memory\n");
			status = -ENOMEM;
			goto err_alloc_tx_buf;
		}
	}
	lrdata->rx_buflen = 0;
	lrdata->tx_buflen = 0;
	lrdata->bufmaxlen = LORA_BUFLEN;
	lrdata->users++;
	init_waitqueue_head(&(lrdata->waitqueue));
	mutex_unlock(&device_list_lock);

	/* Map the data location to the file data pointer. */
	filp->private_data = lrdata;
	/* This a character device, so it is not seekable. */
	nonseekable_open(inode, filp);

	return 0;

err_alloc_tx_buf:
	kfree(lrdata->rx_buf);
	lrdata->rx_buf = NULL;
err_find_dev:
	mutex_unlock(&device_list_lock);

	return status;
}

static int
file_close(struct inode *inode, struct file *filp)
{
	struct lora_struct *lrdata;
	
	pr_debug("lora: close file\n");
	
	lrdata = filp->private_data;

	mutex_lock(&device_list_lock);
	filp->private_data = NULL;
	
	if (lrdata->users > 0)
		lrdata->users--;
	
	/* Last close */
	if (lrdata->users == 0) {
		kfree(lrdata->rx_buf);
		kfree(lrdata->tx_buf);
		lrdata->rx_buf = NULL;
		lrdata->tx_buf = NULL;
	}
	mutex_unlock(&device_list_lock);

	return 0;
}

static ssize_t
file_read(struct file *filp, char __user *buf, size_t size, loff_t *pos)
{
	struct lora_struct *lrdata;

	pr_debug("lora: read file (size=%zu)\n", size);

	lrdata = filp->private_data;

	if (lrdata->ops->read != NULL)
		return lrdata->ops->read(lrdata, buf, size);
	else
		return 0;
}

static ssize_t
file_write(struct file *filp, const char __user *buf, size_t size, loff_t *pos)
{
	struct lora_struct *lrdata;

	pr_debug("lora: write file (size=%zu)\n", size);

	lrdata = filp->private_data;

	if (lrdata->ops->write != NULL)
		return lrdata->ops->write(lrdata, buf, size);
	else
		return 0;
}

static long
file_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;
	int *pval;
	struct lora_struct *lrdata;

	pr_debug("lora: ioctl file (cmd=0x%X)\n", cmd);

	ret = -ENOTTY;
	pval = (void __user *)arg;
	lrdata = filp->private_data;

	/* I/O control by each command. */
	switch (cmd) {
	/* Set & read the state of the LoRa device. */
	case LORA_SET_STATE:
		if (lrdata->ops->setState != NULL)
			ret = lrdata->ops->setState(lrdata, pval);
		break;
	case LORA_GET_STATE:
		if (lrdata->ops->getState != NULL)
			ret = lrdata->ops->getState(lrdata, pval);
		break;
	/* Set & get the carrier frequency. */
	case LORA_SET_FREQUENCY:
		if (lrdata->ops->setFreq != NULL)
			ret = lrdata->ops->setFreq(lrdata, pval);
		break;
	case LORA_GET_FREQUENCY:
		if (lrdata->ops->getFreq != NULL)
			ret = lrdata->ops->getFreq(lrdata, pval);
		break;
	/* Set & get the PA power. */
	case LORA_SET_POWER:
		if (lrdata->ops->setPower != NULL)
			ret = lrdata->ops->setPower(lrdata, pval);
		break;
	case LORA_GET_POWER:
		if (lrdata->ops->getPower != NULL)
			ret = lrdata->ops->getPower(lrdata, pval);
		break;
	/* Set & get the LNA gain. */
	case LORA_SET_LNA:
		if (lrdata->ops->setLNA != NULL)
			ret = lrdata->ops->setLNA(lrdata, pval);
		break;
	case LORA_GET_LNA:
		if (lrdata->ops->getLNA != NULL)
			ret = lrdata->ops->getLNA(lrdata, pval);
		break;
	/* Set the LNA be auto gain control or manual. */
	case LORA_SET_LNAAGC:
		if (lrdata->ops->setLNAAGC != NULL)
			ret = lrdata->ops->setLNAAGC(lrdata, pval);
		break;
	/* Set & get the RF spreading factor. */
	case LORA_SET_SPRFACTOR:
		if (lrdata->ops->setSPRFactor != NULL)
			ret = lrdata->ops->setSPRFactor(lrdata, pval);
		break;
	case LORA_GET_SPRFACTOR:
		if (lrdata->ops->getSPRFactor != NULL)
			ret = lrdata->ops->getSPRFactor(lrdata, pval);
		break;
	/* Set & get the RF bandwith. */
	case LORA_SET_BANDWIDTH:
		if (lrdata->ops->setBW != NULL)
			ret = lrdata->ops->setBW(lrdata, pval);
		break;
	case LORA_GET_BANDWIDTH:
		if (lrdata->ops->getBW != NULL)
			ret = lrdata->ops->getBW(lrdata, pval);
		break;
	/* Get current RSSI. */
	case LORA_GET_RSSI:
		if (lrdata->ops->getRSSI != NULL)
			ret = lrdata->ops->getRSSI(lrdata, pval);
		break;
	/* Get last packet's SNR. */
	case LORA_GET_SNR:
		if (lrdata->ops->getSNR != NULL)
			ret = lrdata->ops->getSNR(lrdata, pval);
		break;
	default:
		ret = -ENOTTY;
	}

	return ret;
}

static unsigned int
file_poll(struct file *filp, poll_table *wait)
{
	struct lora_struct *lrdata;
	unsigned int mask;

	pr_debug("lora: poll file\n");

	lrdata = filp->private_data;
	if (lrdata == NULL)
		return -EBADFD;

	/* Register the file into wait queue for multiplexing. */
	poll_wait(filp, &lrdata->waitqueue, wait);

	/* Check ready to write / read. */
	mask = 0;
	if ((lrdata->ops->ready2write != NULL)
		&& lrdata->ops->ready2write(lrdata))
		mask |= POLLOUT | POLLWRNORM;
	if ((lrdata->ops->ready2read != NULL)
		&& lrdata->ops->ready2read(lrdata))
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

/**
 * lora_device_add - Add a LoRa compatible device into the device list
 * @lrdata:	the LoRa device going to be added
 *
 * Return:	0 / other number for success / failed
 */
int
lora_device_add(struct lora_struct *lrdata)
{
	INIT_LIST_HEAD(&(lrdata->device_entry));

	mutex_lock(&device_list_lock);
	list_add(&(lrdata->device_entry), &device_list);
	mutex_unlock(&device_list_lock);

	return 0;
}

/**
 * lora_device_remove - Remove a LoRa compatible device from the device list
 * @lrdata:	the LoRa device going to be removed
 *
 * Return:	0 / other number for success / failed
 */
int
lora_device_remove(struct lora_struct *lrdata)
{
	mutex_lock(&device_list_lock);
	list_del(&(lrdata->device_entry));
	mutex_unlock(&device_list_lock);

	return 0;
}

static struct file_operations lora_fops = {
	.open 		= file_open,
	.release	= file_close,
	.read		= file_read,
	.write		= file_write,
	.unlocked_ioctl = file_ioctl,
	.poll		= file_poll,
	.llseek		= no_llseek,
};

/**
 * lora_register_driver - Register there is a kind of LoRa driver
 * @driver:	LoRa driver going to be registered
 *
 * Return:	0 / negative number for success / error number
 */
int
lora_register_driver(struct lora_driver *driver)
{
	dev_t dev;
	int alloc_ret, cdev_err;

	pr_debug("lora: register %s\n", driver->name);

	/* Allocate a character device. */
	alloc_ret = alloc_chrdev_region(&dev,
					driver->minor_start,
					driver->num,
					driver->name);
	if (alloc_ret) {
		pr_err("lora: Failed to allocate a character device\n");
		return alloc_ret;
	}
	/* Initial the character device driver. */
	driver->major = MAJOR(dev);
	cdev_init(&(driver->lora_cdev), &lora_fops);
	driver->lora_cdev.owner = driver->owner;
	/* Add the character device driver into system. */
	cdev_err = cdev_add(&(driver->lora_cdev), dev, driver->num);
	if (cdev_err) {
		pr_err("lora: Failed to register a character device\n");
		/* Release the allocated character device. */
		if (alloc_ret == 0) {
			unregister_chrdev_region(dev, driver->num);
		}
		return cdev_err;
	}
	pr_debug("lora: %s driver(major %d) installed\n",
			driver->name, driver->major);

	/* Create device class. */
	driver->lora_class = class_create(driver->owner, driver->name);
	if (IS_ERR(driver->lora_class)) {
		pr_err("lora: Failed to create a class of device\n");
		/* Release the added character device. */
		if (cdev_err == 0)
			cdev_del(&(driver->lora_cdev));
		/* Release the allocated character device. */
		if (alloc_ret == 0)
			unregister_chrdev_region(dev, driver->num);
		return PTR_ERR(driver->lora_class);
	}
	pr_debug("lora: %s class created\n", driver->name);

	return 0;
}

/**
 * lora_unregister_driver - Unregister the LoRa driver
 * @driver:	LoRa driver going to be unregistered
 *
 * Return:	0 / negative number for success / error number
 */
int
lora_unregister_driver(struct lora_driver *driver)
{
	dev_t dev = MKDEV(driver->major, driver->minor_start);
	
	pr_debug("lora: unregister %s\n", driver->name);
	/* Delete device class. */
	class_destroy(driver->lora_class);
	/* Delete the character device driver from system. */
	cdev_del(&(driver->lora_cdev));
	/* Unregister the allocated character device. */
	unregister_chrdev_region(dev, driver->num);
	pr_debug("lora: %s driver removed\n", driver->name);

	return 0;
}
