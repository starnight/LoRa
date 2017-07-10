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

#ifndef __LORA_H__
#define __LORA_H__

#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/mutex.h>

/* I/O control by each command. */
#define LORA_IOC_MAGIC '\x74'

#define LORA_SET_STATE		(_IOW(LORA_IOC_MAGIC,  0, int))
#define LORA_GET_STATE		(_IOR(LORA_IOC_MAGIC,  1, int))
#define LORA_SET_FREQUENCY	(_IOW(LORA_IOC_MAGIC,  2, int))
#define LORA_GET_FREQUENCY	(_IOR(LORA_IOC_MAGIC,  3, int))
#define LORA_SET_POWER		(_IOW(LORA_IOC_MAGIC,  4, int))
#define LORA_GET_POWER		(_IOR(LORA_IOC_MAGIC,  5, int))
#define LORA_SET_LNA		(_IOW(LORA_IOC_MAGIC,  6, int))
#define LORA_GET_LNA		(_IOR(LORA_IOC_MAGIC,  7, int))
#define LORA_SET_LNAAGC		(_IOR(LORA_IOC_MAGIC,  8, int))
#define LORA_SET_SPRFACTOR	(_IOW(LORA_IOC_MAGIC,  9, int))
#define LORA_GET_SPRFACTOR	(_IOR(LORA_IOC_MAGIC, 10, int))
#define LORA_SET_BANDWIDTH	(_IOW(LORA_IOC_MAGIC, 11, int))
#define LORA_GET_BANDWIDTH	(_IOR(LORA_IOC_MAGIC, 12, int))
#define LORA_GET_RSSI		(_IOR(LORA_IOC_MAGIC, 13, int))
#define LORA_GET_SNR		(_IOR(LORA_IOC_MAGIC, 14, int))

/* List the state of the LoRa device. */
#define LORA_STATE_SLEEP	0
#define LORA_STATE_STANDBY	1
#define LORA_STATE_TX		2
#define LORA_STATE_RX		3
#define LORA_STATE_CAD		4

struct lora_struct;

/* The structure lists the LoRa device's operations. */
struct lora_operations {
	/* Set & get the state of the LoRa device. */
	long (*setState)(struct lora_struct *, void __user *);
	long (*getState)(struct lora_struct *, void __user *);
	/* Set & get the carrier frequency. */
	long (*setFreq)(struct lora_struct *, void __user *);
	long (*getFreq)(struct lora_struct *, void __user *);
	/* Set & get the PA power. */
	long (*setPower)(struct lora_struct *, void __user *);
	long (*getPower)(struct lora_struct *, void __user *);
	/* Set & get the LNA gain. */
	long (*setLNA)(struct lora_struct *, void __user *);
	long (*getLNA)(struct lora_struct *, void __user *);
	/* Set LNA be auto gain control or manual. */
	long (*setLNAAGC)(struct lora_struct *, void __user *);
	/* Set & get the RF spreading factor. */
	long (*setSPRFactor)(struct lora_struct *, void __user *);
	long (*getSPRFactor)(struct lora_struct *, void __user *);
	/* Set & get the RF bandwith. */
	long (*setBW)(struct lora_struct *, void __user *);
	long (*getBW)(struct lora_struct *, void __user *);
	/* Get current RSSI. */
	long (*getRSSI)(struct lora_struct *, void __user *);
	/* Get last packet's SNR. */
	long (*getSNR)(struct lora_struct *, void __user *);
	/* Read from the LoRa device's communication. */
	ssize_t (*read)(struct lora_struct *, const char __user *, size_t);
	/* Write to the LoRa device's communication. */
	ssize_t (*write)(struct lora_struct *, const char __user *, size_t);
	/* Is ready to write & read. */
	long (*ready2write)(struct lora_struct *);
	long (*ready2read)(struct lora_struct *);
};

/**
 * struct lora_struct: Master side proxy of an LoRa slave device
 * @devt:		It is a device search key
 * @lora_device:	LoRa controller used with the device
 * @device_entry:	The entry going to be added into the device list
 * @ops:		Handle of LoRa operations interfaces
 * @tx_buf:		Pointer of the TX buffer
 * @rx_buf:		Pointer of the RX buffer
 * @tx_buflen:		The length of the TX buffer
 * @rx_buffer:		The length of the RX buffer
 * @bufmaxlen:		The max length of the TX and RX buffer
 * @users:		How many program use this LoRa device
 * @buf_lock:		The lock to protect the synchroniztion of this structure
 * @waitqueue:		The queue to be hung on the wait table for multiplexing
 */
struct lora_struct {
	dev_t devt;
	void *lora_device;
	struct list_head device_entry;
	struct lora_operations *ops;
	uint8_t *tx_buf;
	uint8_t *rx_buf;
	uint8_t tx_buflen;
	uint8_t rx_buflen;
	uint8_t bufmaxlen;
	uint8_t users;
	struct mutex buf_lock;
	wait_queue_head_t waitqueue;
};

/**
 * struct lora_driver: Host side LoRa driver
 * @name:		Name of the driver to use with this device
 * @major:		Driver's major number
 * @minor_start:	Driver's minor number starts from
 * @num:		The max number of the devices which use this driver
 * @lora_cdev:		The handle lets the devices act as character devices
 * @lora_class:		The class for being registed into file system
 * @owner:		This driver owned by which kernel module
 */
struct lora_driver {
	char *name;
	int major;
	int minor_start;
	int num;
	struct cdev lora_cdev;
	struct class *lora_class;
	struct module *owner;
};

int lora_device_add(struct lora_struct *);
int lora_device_remove(struct lora_struct *);
int lora_register_driver(struct lora_driver *);
int lora_unregister_driver(struct lora_driver *);

#endif
