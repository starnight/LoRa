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
#include <linux/workqueue.h>
#include <linux/netdevice.h>
#include <crypto/hash.h>
#include <crypto/skcipher.h>
#include "lora.h"

#define	LORAWAN_MODULE_NAME	"lorawan"

#define	LRW_UPLINK		0
#define	LRW_DOWNLINK		1

enum {
	LRW_INIT_SS,
	LRW_XMITTING_SS,
	LRW_XMITTED,
	LRW_RX1_SS,
	LRW_RX2_SS,
	LRW_RXTIMEOUT_SS,
	LRW_RXRECEIVED_SS,
	LRW_RETRANSMIT_SS,
};

#define	LRW_MHDR_LEN		1
#define	LRW_FHDR_MAX_LEN	22
#define	LRW_FPORT_LEN		1
#define	LRW_MIC_LEN		4

/**
 * lrw_fhdr - Hold the message's basic information of the frame
 *
 * @mtype:		this message's type
 * @fctrl:		the frame control byte
 * @fcnt:		this message's frame counter value
 * @fopts:		this frame's options field
 * @fopts_len:		the length of the fopts
 */
struct lrw_fhdr {
	u8 mtype;
	u8 fctrl;
	u16 fcnt;
	u8 fopts[15];
	u8 fopts_len;
};

/**
 * lrw_session - LoRaWAN session for Class A end device
 *
 * @lrw_st:		points to the belonging lrw_st
 * @entry:		the entry of the ss_list in lrw_struct
 * @devaddr:		the LoRaWAN device address of this LoRaWAN hardware
 * @fcnt_up:		uplink frame counter
 * @fcnt_down:		downlink frame counter
 * @fport:		the LoRaWAN data message's port field
 * @tx_skb:		points to the TX skb, the frame
 * @rx_skb:		points to the RX skb, the frame
 * @tx_fhdr:		hold the message's basic information of the TX frame
 * @rx_fhdr:		hold the message's basic information of the RX frame
 * @tx_should_ack:	flag for determining the TX which should be acked or not
 * @retry:		retry times for xmitting failed
 * @state:		this session's current state
 * @state_lock:		lock of the session's state
 * @timer:		timing for this session and the state transition
 * @timeout_work:	work if waiting acknowledge time out
 * @rx_delay1:		RX1 delay time in seconds
 * @rx_delay2:		RX2 delay time in seconds
 * @rx1_window:		RX1 window opening time in mini-seconds
 * @rx2_window:		RX2 window opening time in mini-seconds
 * @ack_timeout:	time out time for waiting acknowledge in seconds
 */
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
	spinlock_t state_lock;

	struct timer_list timer;
	struct work_struct timeout_work;
	u16 rx_delay1;
	u16 rx_delay2;
	u16 rx1_window;
	u16 rx2_window;
	u16 ack_timeout;
};

/**
 * lrw_struct - The full LoRaWAN hardware to the LoRa device.
 *
 * @dev:		this LoRa device registed in system
 * @lora_hw:		the LoRa device of this LoRaWAN hardware
 * @ops:		handle of LoRa operations interfaces
 * @rx_skb_list:	the list of received frames
 * @ss_list:		LoRaWAN session list of this LoRaWAN hardware
 * @_cur_ss:		pointer of the current processing session
 * @rx_should_ack:	represent the current session should be acked or not
 * @role:		the role of this LoRaWAN hardware
 * @state:		the state of this LoRaWAN hardware
 * @devaddr:		the LoRaWAN device address of this LoRaWAN hardware
 * @appky:		the Application key
 * @nwkskey:		the Network session key
 * @appskey:		the Application session key
 * @nwks_shash_tfm:	the hash handler for LoRaWAN network session
 * @nwks_skc_tfm:	the crypto handler for LoRaWAN network session
 * @apps_skc_tfm:	the crypto handler for LoRaWAN application session
 * @fcnt_up:		the counter of this LoRaWAN hardware's up frame
 * @fcnt_down:		the counter of this LoRaWAN hardware's down frame
 * @xmit_task:		the xmit task for the current LoRaWAN session
 * @rx_work:		the RX work in workqueue for the current LoRaWAN session
 * @ndev:		points to the emulating network device
 * @_net:		the current network namespace of this LoRaWAN hardware
 */
struct lrw_struct {
	struct device dev;
	struct lora_hw hw;
	struct lora_operations *ops;
	struct sk_buff_head rx_skb_list;
	struct list_head ss_list;
	struct mutex ss_list_lock;
	struct lrw_session *_cur_ss;
	bool rx_should_ack;
	uint8_t role;
	u8 state;

	u8 devaddr[LRW_DEVADDR_LEN];
	u8 appkey[LORA_KEY_LEN];
	u8 nwkskey[LORA_KEY_LEN];
	u8 appskey[LORA_KEY_LEN];
	struct crypto_shash *nwks_shash_tfm;
	struct crypto_skcipher *nwks_skc_tfm;
	struct crypto_skcipher *apps_skc_tfm;

	u16 fcnt_up;
	u16 fcnt_down;

	struct tasklet_struct xmit_task;
	struct work_struct rx_work;

	struct net_device *ndev;
	possible_net_t _net;
};

#define	NETDEV_2_LRW(ndev)	((struct lrw_struct *)netdev_priv(ndev))

struct lrw_session * lrw_alloc_ss(struct lrw_struct *);
void lrw_free_ss(struct lrw_session *);
void lrw_del_ss(struct lrw_session *);
int lora_start_hw(struct lrw_struct *);
void lora_stop_hw(struct lrw_struct *);
void lrw_prepare_tx_frame(struct lrw_session *);
void lrw_xmit(unsigned long);
void lrw_rx_work(struct work_struct *);

int lrw_sock_init(void);
void lrw_sock_exit(void);

struct lrw_addr {
	u8 devaddr[LRW_DEVADDR_LEN];
};

struct lrw_addr_sa {
	int addr_type;
	u8 devaddr[LRW_DEVADDR_LEN];
};

struct sockaddr_lorawan {
	sa_family_t family; /* AF_LORAWAN */
	struct lrw_addr_sa addr_sa;
};

struct lrw_mac_cb {
	int rssi;
	struct lrw_addr addr;
};

#endif
