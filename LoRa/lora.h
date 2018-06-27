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
#include <linux/skbuff.h>
#include <linux/random.h>

/* List the role of the LoRaWAN hardware */
enum {
	LRW_GATEWAY,
	LRW_CLASS_A_NODE,
	LRW_CLASS_B_NODE,
	LRW_CLASS_C_NODE,
};

/* List the regions */
enum {
	LRW_EU863_870,
	LRW_US902_928,
	LRW_CN779_787,
	LRW_EU443,
	LRW_AU915_928,
	LRW_CN470_510,
	LRW_AS923,
	LRW_KR920_923,
	LRW_INDIA865_867,
	LRW_MAX_REGION,
};

enum {
	LRW_LORA,
	LRW_FSK,
};

#define	LRW_UPLINK		0
#define	LRW_DOWNLINK		1

#define	LRW_DEVADDR_LEN		(sizeof(__le32))

/* List the message types of LoRaWAN */
enum {
	LRW_JOIN_REQUEST,
	LRW_JOIN_ACCEPT,
	LRW_UNCONFIRMED_DATA_UP,
	LRW_UNCONFIRMED_DATA_DOWN,
	LRW_CONFIRMED_DATA_UP,
	LRW_CONFIRMED_DATA_DOWN,
	LRW_RFU,
	LRW_PROPRIETARY,
};

enum {
	LRW_ADDR_APPEUI,
	LRW_ADDR_DEVEUI,
	LRW_ADDR_DEVADDR,
};

struct lrw_addr_in {
	int addr_type;
	union {
		u64 app_eui;
		u64 dev_eui;
		u32 devaddr;
	};
};

struct sockaddr_lorawan {
	sa_family_t family; /* AF_LORAWAN */
	struct lrw_addr_in addr_in;
};

/* DEV ioctl() commands */
enum LRW_IOC_CMDS {
	SIOCGLRWSTATE = SIOCDEVPRIVATE,
	SIOCSLRWSTATE,
	SIOCGLRWKEY,
	SIOCSLRWKEY,
	SIOCGLRWFREQ,
	SIOCSLRWFREQ,
	SIOCGLRWBW,
	SIOCSLRWBW,
	SIOCGLRWTXPWR,
	SIOCSLRWTXPWR,
	SIOCGLRWSPRF,
	SIOCSLRWSPRF,
	SIOCGLRWLNA,
	SIOCSLRWLNA,
	SIOCGLRWRSSI,
	SIOCGLRWSNR,
};

/* I/O control by each command */
//#define LRW_IOC_MAGIC '\x74'
//
//#define LRW_SET_STATE			(_IOW(LRW_IOC_MAGIC,  0, int))
//#define LRW_GET_STATE			(_IOR(LRW_IOC_MAGIC,  1, int))
//#define LRW_SET_FREQUENCY		(_IOW(LRW_IOC_MAGIC,  2, int))
//#define LRW_GET_FREQUENCY		(_IOR(LRW_IOC_MAGIC,  3, int))
//#define LRW_SET_POWER			(_IOW(LRW_IOC_MAGIC,  4, int))
//#define LRW_GET_POWER			(_IOR(LRW_IOC_MAGIC,  5, int))
//#define LRW_SET_LNA			(_IOW(LRW_IOC_MAGIC,  6, int))
//#define LRW_GET_LNA			(_IOR(LRW_IOC_MAGIC,  7, int))
//#define LRW_SET_LNAAGC			(_IOR(LRW_IOC_MAGIC,  8, int))
//#define LRW_SET_SPRFACTOR		(_IOW(LRW_IOC_MAGIC,  9, int))
//#define LRW_GET_SPRFACTOR		(_IOR(LRW_IOC_MAGIC, 10, int))
//#define LRW_SET_BANDWIDTH		(_IOW(LRW_IOC_MAGIC, 11, int))
//#define LRW_GET_BANDWIDTH		(_IOR(LRW_IOC_MAGIC, 12, int))
//#define LRW_GET_RSSI			(_IOR(LRW_IOC_MAGIC, 13, int))
//#define LRW_GET_SNR			(_IOR(LRW_IOC_MAGIC, 14, int))

/* List the LoRa device's states of LoRaWAN hardware */
enum {
	LORA_STOP,
	LORA_START,
	LORA_STATE_IDLE,
	LORA_STATE_TX,
	LORA_STATE_RX,
	LORA_STATE_CAD,
};

/**
 * lora_hw - This structure holds the LoRa device of LoRaWAN hardware.
 *
 * @parent:		points to the parent device
 * @priv:		points to the private data
 * @channels:		bits array of RF channels could be used
 * @tx_powers:		points to the emitting RF power array
 * @tx_powers_size:	the size of emitting RF power array in bytes
 * @transmit_power:	the current emitting RF power in mBm
 */
struct lora_hw {
	struct device *parent;
	void *priv;
	u32 channels;
	u8 current_channel;
	s32 *tx_powers;
	size_t tx_powers_size;
	s32 transmit_power;
};

/**
 * lora_operations - The structure lists the LoRa device/interface's operations.
 * These are callback functions for the LoRaWAN module.  LoRa device driver
 * should implement some of them according to the usage.  The unimplemented
 * callback functions must be assigned as NULL.
 *
 * @start:
 *	called when the interface is being up state
 *
 * @stop:
 *	called when the interface is being down state
 *
 * @xmit_async:
 *	called to xmit the data through the interface asynchronously
 *
 * @set_txpower:
 *	called to set xmitting RF power in mBm of the interface
 *
 * @set_frq:
 *	called to set carrier frequency Hz of the interface
 *
 * @set_bw:
 *	called to set RF bandwidth in Hz of the interface
 *
 * @set_mod:
 *	called to set the LoRa device's working mode: LoRa or FSK mode
 *
 * @set_sf:
 *	called to set the CSS modulation's spreading factor of LoRa
 *
 * @start_rx_window:
 *	called to ask the LoRa device open a receiving window
 *
 * @set_state:
 *	called to set the LoRa device's working state
 *
 * @get_state:
 *	called to get the LoRa device's current working state
 */
struct lora_operations {
	int (*start)(struct lora_hw *);
	void (*stop)(struct lora_hw *);

	int (*xmit_async)(struct lora_hw *, struct sk_buff *);
	int (*set_txpower)(struct lora_hw *, s32);
	int (*set_frq)(struct lora_hw *, u32);
	int (*set_bw)(struct lora_hw *, u32);
	int (*set_mod)(struct lora_hw *, u8);
	int (*set_sf)(struct lora_hw *, u8);
	int (*start_rx_window)(struct lora_hw *, u32);

	/* Set & get the state of the LoRa device. */
	int (*set_state)(struct lora_hw *, u8);
	int (*get_state)(struct lora_hw *, u8);
};

struct lora_hw *lora_alloc_hw(size_t, struct lora_operations *);
void lora_free_hw(struct lora_hw *);
int lora_register_hw(struct lora_hw *);
void lora_unregister_hw(struct lora_hw *);
int lora_set_region(struct lora_hw *, u8);
void lora_rx_irqsave(struct lora_hw *, struct sk_buff *);
void lora_xmit_complete(struct lora_hw *, struct sk_buff *);

static inline void lora_random_addr(__le64 *addr)
{
	get_random_bytes(addr, sizeof(__le64));
}

void lora_set_deveui(struct lora_hw *, __le64);
__le64 lora_get_deveui(struct lora_hw *);
void lora_set_appeui(struct lora_hw *, __le64);
__le64 lora_get_appeui(struct lora_hw *);
void lora_set_devaddr(struct lora_hw *, __le32);
__le32 lora_get_devaddr(struct lora_hw *);

enum {
	LORA_APPKEY,
	LORA_NWKSKEY,
	LORA_APPSKEY,
};

#define	LORA_KEY_LEN		16

struct lora_key {
	int type;
	u8 key[LORA_KEY_LEN];
};

int lora_set_key(struct lora_hw *, u8, u8 *, size_t);
int lrw_get_devaddr(struct lora_hw *, u8 *devaddr);

/* Need to find a way to define or assign */
#define	LORAWAN_MTU		20

#define	le32_to_be32(n)		(cpu_to_be32(le32_to_cpu(n)))
#define	be32_to_le32(n)		(cpu_to_le32(be32_to_cpu(n)))
#define	le64_to_be64(n)		(cpu_to_be64(le64_to_cpu(n)))
#define	be64_to_le64(n)		(cpu_to_le64(be64_to_cpu(n)))

#endif
