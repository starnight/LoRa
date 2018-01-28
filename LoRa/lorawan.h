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

#ifndef __LORAWAN_H__
#define __LORAWAN_H__

#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/skbuff.h>
#include <crypto/hash.h>
#include <crypto/skcipher.h>
#include "lora.h"

#define	LRW_UPLINK		0
#define	LRW_DOWNLINK		1

#define	LRW_INIT_SS		0
#define	LRW_XMITTING_SS		1
#define	LRW_XMITTED		2
#define	LRW_RX1_SS		3
#define	LRW_RX2_SS		4
#define	LRW_RXTIMEOUT_SS	5
#define	LRW_RXRECEIVED_SS	6
#define	LRW_RETRANSMIT_SS	7

struct lrw_fhdr {
	u8 mtype;
	u8 fctrl;
	u16 fcnt;
	u8 fopts[15];
	u8 fopts_len;
};

struct lrw_session {
	struct lrw_struct *lrw_st;
	struct list_head entry;

	u8 devaddr[LRW_DEVADDR_LEN];
	u16 fcnt_up;
	u16 fcnt_down;
	u8 fport;
	struct sk_buff *tx_skb;
	struct sk_buff *rx_skb;
	struct lrw_fhdr tx_fhdr;
	struct lrw_fhdr rx_fhdr;

	bool tx_should_ack;
	u8 retry;
	u8 state;
	struct spinlock_t state_lock;

	struct timer_list timer;
	struct work_struct timeout_work;
	u16 rx_delay1; // seconds
	u16 rx_delay2; // seconds
	u16 rx1_window; // mini-seconds
	u16 rx2_window; // mini-seconds
	u16 ack_timeout; // seconds

	u8 *appkey;
	u8 *nwkskey;
	u8 *appskey;
};

/**
 * struct lrw_struct: Master side proxy of an LoRa slave device
 * @devt:		It is a device search key
 * @lrw_device:	LoRa controller used with the device
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
struct lrw_struct {
	dev_t devt;
	struct device *dev;
	struct lora_hw hw;
	struct lora_operations *ops;
	struct list_head device_entry;
	struct list_head ss_list;
	struct lrw_session *_cur_ss;
	struct sk_buff_head rx_skb_list;
	struct sk_buff_head sent_tx_skb_list;
	struct mutex ss_list_lock;
	uint8_t users;
	uint8_t role;
	wait_queue_head_t waitqueue;
	struct cdev lrw_cdev;
	bool rx_should_ack;

	u8 devaddr[LRW_DEVADDR_LEN];
	u8 appkey[16];
	u8 nwkskey[16];
	u8 appskey[16];
	struct crypto_shash *nwks_shash_tfm;
	struct crypto_skcipher *nwks_skc_tfm;
	struct crypto_skcipher *apps_skc_tfm;

	struct tasklet_struct xmit_task;
	struct work_struct rx_work;
};

#endif
