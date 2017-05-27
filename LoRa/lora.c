#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/slab.h>

#include "lora.h"

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

#define LORA_BUFLEN	123

static int file_open(struct inode *inode, struct file *filp) {
	struct lora_data *lrdata;
	int status = -ENXIO;

	printk(KERN_DEBUG "lora: open\n");
	
	mutex_lock(&device_list_lock);
	/* Use device_entry to find the lora date with matched dev_t in inode. */
	list_for_each_entry(lrdata, &device_list, device_entry) {
		if(lrdata->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if(status) {
		pr_debug("lora: nothing for minor %d\n", iminor(inode));
		goto err_find_dev;
	}

	/* Have the RX/TX memory buffer. */
	if(!(lrdata->rx_buf)) {
		lrdata->rx_buf = kzalloc(LORA_BUFLEN, GFP_KERNEL);
		if(!(lrdata->rx_buf)) {
			pr_debug("lora: no more memory\n");
			status = -ENOMEM;
			goto err_find_dev;
		}
	}
	if(!(lrdata->tx_buf)) {
		lrdata->tx_buf = kzalloc(LORA_BUFLEN, GFP_KERNEL);
		if(!(lrdata->tx_buf)) {
			pr_debug("lora: no more memory\n");
			status = -ENOMEM;
			goto err_alloc_tx_buf;
		}
	}
	lrdata->rx_buflen = 0;
	lrdata->tx_buflen = 0;
	lrdata->bufmaxlen = LORA_BUFLEN;
	lrdata->users++;
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

static int file_close(struct inode *inode, struct file *filp) {
	struct lora_data *lrdata;
	
	printk(KERN_DEBUG "lora: close\n");
	
	lrdata = filp->private_data;

	mutex_lock(&device_list_lock);
	filp->private_data = NULL;
	
	if(lrdata->users > 0)
		lrdata->users--;
	
	/* Last close */
	if(lrdata->users == 0) {
		kfree(lrdata->rx_buf);
		kfree(lrdata->tx_buf);
		lrdata->rx_buf = NULL;
		lrdata->tx_buf = NULL;
	}
	mutex_unlock(&device_list_lock);

	return 0;
}

static ssize_t file_read(struct file *filp, char __user *buf, size_t size, loff_t *f_pos) {
	struct lora_data *lrdata;

	printk(KERN_DEBUG "lora: read (size=%zu)\n", size);

	lrdata = filp->private_data;

	if(lrdata->ops->read != NULL)
		return lrdata->ops->read(lrdata, buf, size);
	else
		return 0;
}

static ssize_t file_write(struct file *filp, const char __user *buf, size_t size, loff_t *f_pos) {
	struct lora_data *lrdata;

	printk(KERN_DEBUG "lora: write (size=%zu)\n", size);

	lrdata = filp->private_data;

	if(lrdata->ops->write != NULL) {
		return lrdata->ops->write(lrdata, buf, size);
	}
	else
		return 0;
}

static long file_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
	long ret;
	int *pval;
	struct lora_data *lrdata;

	ret = -ENOTTY;
	pval = (void __user *)arg;
	lrdata = filp->private_data;

	/* I/O control by each command. */
	switch(cmd) {
		/* Set & read the state of the LoRa device. */
		case LORA_SET_STATE:
			if(lrdata->ops->setState != NULL)
				ret = lrdata->ops->setState(lrdata, pval);
			break;
		case LORA_GET_STATE:
			if(lrdata->ops->getState != NULL)
				ret = lrdata->ops->getState(lrdata, pval);
			break;
		/* Set & get the carrier frequency. */
		case LORA_SET_FREQUENCY:
			if(lrdata->ops->setFreq != NULL)
				ret = lrdata->ops->setFreq(lrdata, pval);
			break;
		case LORA_GET_FREQUENCY:
			if(lrdata->ops->getFreq != NULL)
				ret = lrdata->ops->getFreq(lrdata, pval);
			break;
		/* Set & get the PA power. */
		case LORA_SET_POWER:
			if(lrdata->ops->setPower != NULL)
				ret = lrdata->ops->setPower(lrdata, pval);
			break;
		case LORA_GET_POWER:
			if(lrdata->ops->getPower != NULL)
				ret = lrdata->ops->getPower(lrdata, pval);
			break;
		/* Set & get the RF spreading factor. */
		case LORA_SET_SPRFACTOR:
			if(lrdata->ops->setSPRFactor != NULL)
				ret = lrdata->ops->setSPRFactor(lrdata, pval);
			break;
		case LORA_GET_SPRFACTOR:
			if(lrdata->ops->getSPRFactor != NULL)
				ret = lrdata->ops->getSPRFactor(lrdata, pval);
			break;
		/* Set & get the RF bandwith. */
		case LORA_SET_BANDWIDTH:
			if(lrdata->ops->setBW != NULL)
				ret = lrdata->ops->setBW(lrdata, pval);
			break;
		case LORA_GET_BANDWIDTH:
			if(lrdata->ops->getBW != NULL)
				ret = lrdata->ops->getBW(lrdata, pval);
			break;
		/* Get current RSSI. */
		case LORA_GET_RSSI:
			if(lrdata->ops->getRSSI != NULL)
				ret = lrdata->ops->getRSSI(lrdata, pval);
			break;
		/* Get last packet's SNR. */
		case LORA_GET_SNR:
			if(lrdata->ops->getSNR != NULL)
				ret = lrdata->ops->getSNR(lrdata, pval);
			break;
		default:
			ret = -ENOTTY;
	}

	return ret;
}

/* Add an lora compatible device. */
static int lora_device_add(struct lora_data *lrdata) {
	INIT_LIST_HEAD(&(lrdata->device_entry));

	mutex_lock(&device_list_lock);
	list_add(&(lrdata->device_entry), &device_list);
	mutex_unlock(&device_list_lock);

	return 0;
}
EXPORT_SYMBOL(lora_device_add);

/* Remove an lora compatible device. */
static int lora_device_remove(struct lora_data *lrdata) {
	mutex_lock(&device_list_lock);
	list_del(&(lrdata->device_entry));
	mutex_unlock(&device_list_lock);

	return 0;
}
EXPORT_SYMBOL(lora_device_remove);

static struct file_operations lora_fops = {
	.open 		= file_open,
	.release	= file_close,
	.read		= file_read,
	.write		= file_write,
	.unlocked_ioctl = file_ioctl,
	.llseek		= no_llseek,
};

/* Register there is a kind of lora driver. */
static int lora_register_driver(struct lora_driver *driver) {
	dev_t dev;
	int alloc_ret, cdev_err;

	printk(KERN_DEBUG "lora: register %s\n", driver->name);

	/* Allocate a character device. */
	alloc_ret = alloc_chrdev_region(&dev,
					driver->minor_start,
					driver->num,
					driver->name);
	if(alloc_ret) {
		printk(KERN_DEBUG "lora: Failed to allocate a character device\n");
		return alloc_ret;
	}
	/* Initial the character device driver. */
	driver->major = MAJOR(dev);
	cdev_init(&(driver->lora_cdev), &lora_fops);
	driver->lora_cdev.owner = driver->owner;
	/* Add the character device driver into system. */
	cdev_err = cdev_add(&(driver->lora_cdev), dev, driver->num);
	if(cdev_err) {
		printk(KERN_DEBUG "lora: Failed to register a character device\n");
		/* Release the allocated character device. */
		if(alloc_ret == 0) {
			unregister_chrdev_region(dev, driver->num);
		}
		return cdev_err;
	}
	printk(KERN_DEBUG "lora: %s driver(major %d) installed.\n", driver->name, driver->major);

	/* Create device class. */
	driver->lora_class = class_create(driver->owner, driver->name);
	if(IS_ERR(driver->lora_class)) {
		printk(KERN_DEBUG "lora: Failed to create a class of device.\n");
		/* Release the added character device. */
		if(cdev_err == 0)
			cdev_del(&(driver->lora_cdev));
		/* Release the allocated character device. */
		if(alloc_ret == 0)
			unregister_chrdev_region(dev, driver->num);
		return -1;
	}
	printk(KERN_DEBUG "lora: %s class created.\n", driver->name);

	return 0;
}
EXPORT_SYMBOL(lora_register_driver);

/* Unregister the lora driver. */
static int lora_unregister_driver(struct lora_driver *driver) {
	dev_t dev = MKDEV(driver->major, driver->minor_start);
	
	printk(KERN_DEBUG "lora: unregister %s\n", driver->name);
	/* Delete device class. */
	class_destroy(driver->lora_class);
	/* Delete the character device driver from system. */
	cdev_del(&(driver->lora_cdev));
	/* Unregister the allocated character device. */
	unregister_chrdev_region(dev, driver->num);
	printk(KERN_DEBUG "lora: %s driver removed.\n", driver->name);

	return 0;
}
EXPORT_SYMBOL(lora_unregister_driver);

MODULE_AUTHOR("JianHong Pan, <starnight@g.ncu.edu.tw>");
MODULE_DESCRIPTION("User mode LoRa device interface");
MODULE_LICENSE("Dual BSD/GPL");
