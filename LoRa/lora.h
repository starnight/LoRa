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

#ifndef __LORA_H__
#define __LORA_H__

#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/skbuff.h>

/* List the role of the LoRaWAN hardware */
#define	LRW_GATEWAY			0x0
#define	LRW_CLASS_A_NODE		0x1
#define	LRW_CLASS_B_NODE		0x2
#define	LRW_CLASS_C_NODE		0x3

#define	LRW_DEVADDR_LEN			4

/* List the message types of LoRaWAN */
#define	LRW_JOIN_REQUEST		0x0
#define LRW_JOIN_ACCEPT			0x1
#define	LRW_UNCONFIRMED_DATA_UP		0x2
#define	LRW_UNCONFIRMED_DATA_DOWN	0x3
#define	LRW_CONFIRMED_DATA_UP		0x4
#define	LRW_CONFIRMED_DATA_DOWN		0x5
#define	LRW_PROPRIETARY			0x7

/* I/O control by each command. */
#define LRW_IOC_MAGIC '\x74'

#define LRW_SET_STATE			(_IOW(LRW_IOC_MAGIC,  0, int))
#define LRW_GET_STATE			(_IOR(LRW_IOC_MAGIC,  1, int))
#define LRW_SET_FREQUENCY		(_IOW(LRW_IOC_MAGIC,  2, int))
#define LRW_GET_FREQUENCY		(_IOR(LRW_IOC_MAGIC,  3, int))
#define LRW_SET_POWER			(_IOW(LRW_IOC_MAGIC,  4, int))
#define LRW_GET_POWER			(_IOR(LRW_IOC_MAGIC,  5, int))
#define LRW_SET_LNA			(_IOW(LRW_IOC_MAGIC,  6, int))
#define LRW_GET_LNA			(_IOR(LRW_IOC_MAGIC,  7, int))
#define LRW_SET_LNAAGC			(_IOR(LRW_IOC_MAGIC,  8, int))
#define LRW_SET_SPRFACTOR		(_IOW(LRW_IOC_MAGIC,  9, int))
#define LRW_GET_SPRFACTOR		(_IOR(LRW_IOC_MAGIC, 10, int))
#define LRW_SET_BANDWIDTH		(_IOW(LRW_IOC_MAGIC, 11, int))
#define LRW_GET_BANDWIDTH		(_IOR(LRW_IOC_MAGIC, 12, int))
#define LRW_GET_RSSI			(_IOR(LRW_IOC_MAGIC, 13, int))
#define LRW_GET_SNR			(_IOR(LRW_IOC_MAGIC, 14, int))

/* List the state of the LoRa hardware. */
#define LORA_STATE_IDLE			0
#define LORA_STATE_TX			1
#define LORA_STATE_RX1			2
#define	LORA_STATE_RX2			3
#define LORA_STATE_CAD			4

struct lora_hw {
	struct device *parent;
	void *priv;
	u32 channels;
	u8 current_channel;
	s32 *tx_powers;
	size_t tx_powers_size;
	s32 transmit_power;
};

/* The structure lists the LoRa device's operations. */
struct lora_operations {
//	/* Set & get the state of the LoRa device. */
//	long (*setState)(struct lora_hw *, void __user *);
//	long (*getState)(struct lora_hw *, void __user *);
//	/* Set & get the carrier frequency. */
//	long (*setFreq)(struct lora_hw *, void __user *);
//	long (*getFreq)(struct lora_hw *, void __user *);
//	/* Set & get the PA power. */
//	long (*setPower)(struct lora_hw *, void __user *);
//	long (*getPower)(struct lora_hw *, void __user *);
//	/* Set & get the LNA gain. */
//	long (*setLNA)(struct lora_hw *, void __user *);
//	long (*getLNA)(struct lora_hw *, void __user *);
//	/* Set LNA be auto gain control or manual. */
//	long (*setLNAAGC)(struct lora_hw *, void __user *);
//	/* Set & get the RF spreading factor. */
//	long (*setSPRFactor)(struct lora_hw *, void __user *);
//	long (*getSPRFactor)(struct lora_hw *, void __user *);
//	/* Set & get the RF bandwith. */
//	long (*setBW)(struct lora_hw *, void __user *);
//	long (*getBW)(struct lora_hw *, void __user *);
//	/* Get current RSSI. */
//	long (*getRSSI)(struct lora_hw *, void __user *);
//	/* Get last packet's SNR. */
//	long (*getSNR)(struct lora_hw *, void __user *);

	int (*start)(struct lora_hw *);
	void (*stop)(struct lora_hw *);
	/* Write to the LoRa device's communication. */
	int (*xmit_async)(struct lora_hw *, struct sk_buff *);
	int (*set_txpower)(struct lora_hw *, s32);
	int (*set_frq)(struct lora_hw *, u32);
	int (*set_bw)(struct lora_hw *, u32);
	int (*set_mod)(struct lora_hw *, u8);
	int (*set_sf)(struct lora_hw *, u8);
	int (*start_rx1_window)(struct lora_hw *, u32);
	int (*start_rx2_window)(struct lora_hw *, u32);
	int (*set_state)(struct lora_hw *, u8);
};

/**
 * struct lrw_driver: Host side LoRa driver
 * @name:		Name of the driver to use with this device
 * @major:		Driver's major number
 * @minor_start:	Driver's minor number starts from
 * @num:		The max number of the devices which use this driver
 * @lrw_cdev:		The handle lets the devices act as character devices
 * @lrw_class:		The class for being registed into file system
 * @owner:		This driver owned by which kernel module
 */
struct lrw_driver {
	char *name;
	int major;
	int minor_start;
	int num;
	struct cdev lrw_cdev;
	struct class *lrw_class;
	struct module *owner;
};

struct lora_hw *lora_alloc_hw(size_t, struct lora_operations *);
void lora_free_hw(struct lora_hw *);
int lora_register_hw(struct lora_hw *);
void lora_unregister_hw(struct lora_hw *);
void lora_rx_irqsave(struct lora_hw *, struct sk_buff *);
void lora_xmit_complete(struct lora_hw *, struct sk_buff *);

#define	LORA_APPKEY		0
#define	LORA_NWKSKEY		1
#define	LORA_APPSKEY		2
#define	LORA_KEY_LEN		16
int lora_set_key(struct lora_hw *, u8, u8 *, size_t);
int lrw_get_devaddr(struct lora_hw *, u8 *devaddr);

#endif
