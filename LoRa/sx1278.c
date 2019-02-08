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

#include <linux/module.h>
#include <linux/timer.h>
#include <linux/device.h>
#include <linux/acpi.h>
#include <linux/of_device.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <net/mac802154.h>

/*------------------------------ LoRa Functions ------------------------------*/

#ifndef F_XOSC
#define F_XOSC		32000000
#endif
static u32 xosc_frq = F_XOSC;
module_param(xosc_frq, uint, 0000);
MODULE_PARM_DESC(xosc_frq, "Crystal oscillator frequency of the LoRa chip");

#define	__POW_2_19	0x80000

#ifndef SX127X_SPRF
#define SX127X_SPRF	512
#endif
static u32 sprf = SX127X_SPRF;
module_param(sprf, uint, 0000);
MODULE_PARM_DESC(sprf, "Spreading factor of Chirp Spread Spectrum modulation");

#ifndef SX127X_RX_BYTE_TIMEOUT
#define SX127X_RX_BYTE_TIMEOUT	1023
#endif
static u32 rx_timeout = SX127X_RX_BYTE_TIMEOUT;
module_param(rx_timeout, uint, 0000);
MODULE_PARM_DESC(rx_timeout, "RX time-out value as number of symbols");

/* SX127X Registers addresses */
#define SX127X_REG_FIFO				0x00
#define SX127X_REG_OP_MODE			0x01
#define SX127X_REG_FRF_MSB			0x06
#define SX127X_REG_FRF_MID			0x07
#define SX127X_REG_FRF_LSB			0x08
#define SX127X_REG_PA_CONFIG			0x09
#define SX127X_REG_PA_RAMP			0x0A
#define SX127X_REG_OCP				0x0B
#define SX127X_REG_LNA				0x0C
#define SX127X_REG_FIFO_ADDR_PTR		0x0D
#define SX127X_REG_FIFO_TX_BASE_ADDR		0x0E
#define SX127X_REG_FIFO_RX_BASE_ADDR		0x0F
#define SX127X_REG_FIFO_RX_CURRENT_ADDR		0x10
#define SX127X_REG_IRQ_FLAGS_MASK		0x11
#define SX127X_REG_IRQ_FLAGS			0x12
#define SX127X_REG_RX_NB_BYTES			0x13
#define SX127X_REG_RX_HEADER_CNT_VALUE_MSB	0x14
#define SX127X_REG_RX_HEADER_CNT_VALUE_LSB	0x15
#define SX127X_REG_RX_PACKET_CNT_VALUE_MSB	0x16
#define SX127X_REG_RX_PACKET_CNT_VALUE_LSB	0x17
#define SX127X_REG_MODEM_STAT			0x18
#define SX127X_REG_PKT_SNR_VALUE		0x19
#define SX127X_REG_PKT_RSSI_VALUE		0x1A
#define SX127X_REG_RSSI_VALUE			0x1B
#define SX127X_REG_HOP_CHANNEL			0x1C
#define SX127X_REG_MODEM_CONFIG1		0x1D
#define SX127X_REG_MODEM_CONFIG2		0x1E
#define SX127X_REG_SYMB_TIMEOUT_LSB		0x1F
#define SX127X_REG_PREAMBLE_MSB			0x20
#define SX127X_REG_PREAMBLE_LSB			0x21
#define SX127X_REG_PAYLOAD_LENGTH		0x22
#define SX127X_REG_MAX_PAYLOAD_LENGTH		0x23
#define SX127X_REG_HOP_PERIOD			0x24
#define SX127X_REG_FIFO_RX_BYTE_ADDR		0x25
#define SX127X_REG_MODEM_CONFIG3		0x26
#define SX127X_REG_FEI_MSB			0x28
#define SX127X_REG_FEI_MID			0x29
#define SX127X_REG_FEI_LSB			0x2A
#define SX127X_REG_RSSI_WIDEBAND		0x2C
#define SX127X_REG_DETECT_OPTIMIZE		0x31
#define SX127X_REG_INVERT_IRQ			0x33
#define SX127X_REG_DETECTION_THRESHOLD		0x37
#define SX127X_REG_SYNC_WORD			0x39
#define SX127X_REG_VERSION			0x42
#define SX127X_REG_TCXO				0x4B
#define SX127X_REG_PA_DAC			0x4D
#define SX127X_REG_FORMER_TEMP			0x5B
#define SX127X_REG_AGC_REF			0x61
#define SX127X_REG_AGC_THRESH1			0x62
#define SX127X_REG_AGC_THRESH2			0x63
#define SX127X_REG_AGC_THRESH3			0x64
#define SX127X_REG_PLL				0x70
#define SX127X_MAX_REG				SX127X_REG_PLL

/* SX127X's operating states in LoRa mode */
#define SX127X_SLEEP_MODE			0x00
#define SX127X_STANDBY_MODE			0x01
#define SX127X_FSTX_MODE			0x02
#define SX127X_TX_MODE				0x03
#define SX127X_FSRX_MODE			0x04
#define SX127X_RXCONTINUOUS_MODE		0x05
#define SX127X_RXSINGLE_MODE			0x06
#define SX127X_CAD_MODE				0x07

/* SX127X's IRQ flags in LoRa mode */
#define SX127X_FLAG_RXTIMEOUT			0x80
#define	SX127X_FLAG_RXDONE			0x40
#define SX127X_FLAG_PAYLOADCRCERROR		0x20
#define SX127X_FLAG_VALIDHEADER			0x10
#define SX127X_FLAG_TXDONE			0x08
#define SX127X_FLAG_CADDONE			0x04
#define SX127X_FLAG_FHSSCHANGECHANNEL		0x02
#define SX127X_FLAG_CADDETECTED			0x01

/* SX127X's IRQ flags' mask for output pins in LoRa mode */
#define SX127X_FLAGMASK_RXTIMEOUT		0x80
#define	SX127X_FLAGMASK_RXDONE			0x40
#define SX127X_FLAGMASK_PAYLOADCRCERROR		0x20
#define SX127X_FLAGMASK_VALIDHEADER		0x10
#define SX127X_FLAGMASK_TXDONE			0x08
#define SX127X_FLAGMASK_CADDONE			0x04
#define SX127X_FLAGMASK_FHSSCHANGECHANNEL	0x02
#define SX127X_FLAGMASK_CADDETECTED		0x01

/* SX127X's RX/TX FIFO base address */
#define SX127X_FIFO_RX_BASE_ADDRESS		0x00
#define SX127X_FIFO_TX_BASE_ADDRESS		0x80

struct sx1278_phy {
	struct ieee802154_hw *hw;
	struct regmap *map;

	bool suspended;
	u8 opmode;
	struct timer_list timer;
	struct work_struct irqwork;
	/* Lock the RX and TX actions. */
	spinlock_t buf_lock;
	struct sk_buff *tx_buf;
	u8 tx_delay;
	bool one_to_be_sent;
	bool post_tx_done;
	bool is_busy;
};

/**
 * sx127X_read_version - Get LoRa device's chip version
 * @map:	the device as a regmap to communicate with
 *
 * Return:	Positive / negtive values for version code / failed
 *		Version code:	bits 7-4 full version number,
 *				bits 3-0 metal mask revision number
 */
int
sx127X_read_version(struct regmap *map)
{
	u8 v;
	int status;

	status = regmap_raw_read(map, SX127X_REG_VERSION, &v, 1);

	if ((status == 0) && (v > 0) && (v < 0xFF))
		status = v;
	else
		status = -ENODEV;

	return status;
}

/**
 * sx127X_set_mode - Set LoRa device's mode register
 * @map:	the device as a regmap to communicate with
 * @op_mode:	LoRa device's operation mode register value
 */
void
sx127X_set_mode(struct regmap *map, u8 op_mode)
{
	regmap_raw_write(map, SX127X_REG_OP_MODE, &op_mode, 1);
}

/**
 * sx127X_get_mode - Get LoRa device's mode register
 * @map:	the device as a regmap to communicate with
 *
 * Return:	LoRa device's register value
 */
u8
sx127X_get_mode(struct regmap *map)
{
	u8 op_mode;

	/* Get original OP Mode register. */
	regmap_raw_read(map, SX127X_REG_OP_MODE, &op_mode, 1);

	return op_mode;
}

/**
 * sx127X_set_state - Set LoRa device's operating state
 * @map:	the device as a regmap to communicate with
 * @st:		LoRa device's operating state going to be assigned
 */
void
sx127X_set_state(struct regmap *map, u8 st)
{
	u8 op_mode;

	/* Get original OP Mode register. */
	op_mode = sx127X_get_mode(map);
	/* Set device to designated state. */
	op_mode = (op_mode & 0xF8) | (st & 0x07);
	regmap_raw_write(map, SX127X_REG_OP_MODE, &op_mode, 1);
}

/**
 * sx127X_get_state - Get LoRa device's operating state
 * @map:	the device as a regmap to communicate with
 *
 * Return:	LoRa device's operating state
 */
u8
sx127X_get_state(struct regmap *map)
{
	u8 op_mode;

	op_mode = sx127X_get_mode(map) & 0x07;

	return op_mode;
}

/**
 * sx127X_set_lorafrq - Set RF frequency
 * @map:	the device as a regmap to communicate with
 * @fr:		RF frequency going to be assigned in Hz
 */
void
sx127X_set_lorafrq(struct regmap *map, u32 fr)
{
	u64 frt;
	u8 buf[3];
	s8 i;
	u32 f_xosc;
	u8 op_mode;

#ifdef CONFIG_OF
	/* Set the LoRa module's crystal oscillator's clock if OF is defined. */
	struct device_node *of_node = (regmap_get_device(map))->of_node;

	if (of_property_read_u32(of_node, "clock-frequency", &f_xosc))
		f_xosc = xosc_frq;
#else
	f_xosc = xosc_frq;
#endif

	frt = (uint64_t)fr * (uint64_t)__POW_2_19;
	do_div(frt, f_xosc);

	for (i = 2; i >= 0; i--)
		buf[i] = do_div(frt, 256);

	op_mode = sx127X_get_mode(map);
	/* Set Low/High frequency bit. */
	if (fr >= 779000000)
		op_mode &= ~0x8;
	else if (fr <= 525000000)
		op_mode |= 0x8;
	sx127X_set_state(map, SX127X_SLEEP_MODE);
	regmap_raw_write(map, SX127X_REG_FRF_MSB, buf, 3);
	sx127X_set_mode(map, op_mode);
}

/**
 * sx127X_get_lorafrq - Get RF frequency
 * @map:	the device as a regmap to communicate with
 *
 * Return:	RF frequency in Hz
 */
u32
sx127X_get_lorafrq(struct regmap *map)
{
	u64 frt = 0;
	u8 buf[3];
	u8 i;
	int status;
	u32 fr;
	u32 f_xosc;

#ifdef CONFIG_OF
	/* Set the LoRa module's crystal oscillator's clock if OF is defined. */
	struct device_node *of_node = (regmap_get_device(map))->of_node;

	if (of_property_read_u32(of_node, "clock-frequency", &f_xosc))
		f_xosc = xosc_frq;
#else
	f_xosc = xosc_frq;
#endif

	status = regmap_raw_read(map, SX127X_REG_FRF_MSB, buf, 3);
	if (status < 0)
		return 0.0;

	for (i = 0; i <= 2; i++)
		frt = frt * 256 + buf[i];

	fr =  frt * f_xosc / __POW_2_19;

	return fr;
}

/**
 * sx127X_set_lorapower - Set RF output power
 * @map:	the device as a regmap to communicate with
 * @pout:	RF output power going to be assigned in dbm
 */
void
sx127X_set_lorapower(struct regmap *map, s32 pout)
{
	u8 pacf;
	u8 boost;
	u8 output_power;
	s32 pmax;

	if (pout > 14) {
		/* Pout > 14dbm */
		boost = 1;
		pmax = 7;
		output_power = pout - 2;
	} else if (pout < 0) {
		/* Pout < 0dbm */
		boost = 0;
		pmax = 2;
		output_power = 3 + pout;
	} else {
		/* 0dbm <= Pout <= 14dbm */
		boost = 0;
		pmax = 7;
		output_power = pout;
	}

	pacf = (boost << 7) | (pmax << 4) | (output_power);
	regmap_raw_write(map, SX127X_REG_PA_CONFIG, &pacf, 1);
}

/**
 * sx127X_get_lorapower - Get RF output power
 * @map:	the device as a regmap to communicate with
 *
 * Return:	RF output power in dbm
 */
s32
sx127X_get_lorapower(struct regmap *map)
{
	u8 pac;
	u8 boost;
	s32 output_power;
	s32 pmax;
	s32 pout;

	regmap_raw_read(map, SX127X_REG_PA_CONFIG, &pac, 1);
	boost = (pac & 0x80) >> 7;
	output_power = pac & 0x0F;
	if (boost) {
		pout = 2 + output_power;
	} else {
		/* Power max should be pmax/10.  It is 10 times for now. */
		pmax = (108 + 6 * ((pac & 0x70) >> 4));
		pout = (pmax - (150 - output_power * 10)) / 10;
	}

	return pout;
}

/**
 * sx127X_dbm2mbm - dbm to mbm unit conversion
 * @dbm:	the value in dbm
 *
 * Return:	the value in mbm
 */
#define sx127X_dbm2mbm(dbm)	(dbm * 100)

/**
 * sx127X_mbm2dbm - mbm to dbm unit conversion
 * @mbm:	the value in mbm
 *
 * Return:	the value in dbm
 */
#define sx127X_mbm2dbm(mbm)	(mbm / 100)

s8 lna_gain[] = {
	 0,
	-6,
	-12,
	-24,
	-26,
	-48
};

/**
 * sx127X_set_loralna - Set RF LNA gain
 * @map:	the device as a regmap to communicate with
 * @db:		RF LNA gain going to be assigned in db
 */
void
sx127X_set_loralna(struct regmap *map, s32 db)
{
	u8 i, g;
	u8 lnacf;

	for (i = 0; i < 5; i++) {
		if (lna_gain[i] <= db)
			break;
	}
	g = i + 1;

	regmap_raw_read(map, SX127X_REG_LNA, &lnacf, 1);
	lnacf = (lnacf & 0x1F) | (g << 5);
	regmap_raw_write(map, SX127X_REG_LNA, &lnacf, 1);
}

/**
 * sx127X_get_loralna - Get RF LNA gain
 * @map:	the device as a regmap to communicate with
 *
 * Return:	RF LNA gain db
 */
s32
sx127X_get_loralna(struct regmap *map)
{
	s32 db;
	s8 i, g;
	u8 lnacf;

	regmap_raw_read(map, SX127X_REG_LNA, &lnacf, 1);
	g = (lnacf >> 5);
	i = g - 1;
	db = lna_gain[i];

	return db;
}

/**
 * sx127X_set_loralnaagc - Set RF LNA go with auto gain control or manual
 * @map:	the device as a regmap to communicate with
 * @yesno:	1 / 0 for auto gain control / manual
 */
void
sx127X_set_loralnaagc(struct regmap *map, s32 yesno)
{
	u8 mcf3;

	regmap_raw_read(map, SX127X_REG_MODEM_CONFIG3, &mcf3, 1);
	mcf3 = (yesno) ? (mcf3 | 0x04) : (mcf3 & (~0x04));
	regmap_raw_write(map, SX127X_REG_MODEM_CONFIG3, &mcf3, 1);
}

/**
 * sx127X_get_loraallflag - Get all of the LoRa device IRQ flags' current state
 * @map:	the device as a regmap to communicate with
 *
 * Return:	All of the LoRa device's IRQ flags' current state in a byte
 */
u8
sx127X_get_loraallflag(struct regmap *map)
{
	u8 flags;

	regmap_raw_read(map, SX127X_REG_IRQ_FLAGS, &flags, 1);

	return flags;
}

/**
 * sx127X_get_loraallflag - Get interested LoRa device IRQ flag's current state
 * @map:	the device as a regmap to communicate with
 * @f:		the interested LoRa device's IRQ flag
 *
 * Return:	The interested LoRa device's IRQ flag's current state in a byte
 */
#define sx127X_get_loraflag(map, f)	(sx127X_get_loraallflag(map) & (f))

/**
 * sx127X_clear_loraflag - Clear designated LoRa device's IRQ flag
 * @map:	the device as a regmap to communicate with
 * @f:		flags going to be cleared
 */
void
sx127X_clear_loraflag(struct regmap *map, u8 f)
{
	u8 flag;

	/* Get oiginal flag. */
	flag = sx127X_get_loraallflag(map);
	/* Set the designated bits of the flag. */
	flag |= f;
	regmap_raw_write(map, SX127X_REG_IRQ_FLAGS, &flag, 1);
}

/**
 * sx127X_clear_loraallflag - Clear designated LoRa device's all IRQ flags
 * @map:	the device as a regmap to communicate with
 */
#define sx127X_clear_loraallflag(spi)	sx127X_clear_loraflag(spi, 0xFF)

/**
 * sx127X_set_lorasprf - Set the RF modulation's spreading factor
 * @map:	the device as a regmap to communicate with
 * @c_s:	Spreading factor in chips / symbol
 */
void
sx127X_set_lorasprf(struct regmap *map, u32 c_s)
{
	u8 sf;
	u8 mcf2;

	for (sf = 6; sf < 12; sf++) {
		if (c_s == ((u32)1 << sf))
			break;
	}

	regmap_raw_read(map, SX127X_REG_MODEM_CONFIG2, &mcf2, 1);
	mcf2 = (mcf2 & 0x0F) | (sf << 4);
	regmap_raw_write(map, SX127X_REG_MODEM_CONFIG2, &mcf2, 1);
}

/**
 * sx127X_get_lorasprf - Get the RF modulation's spreading factor
 * @map:	the device as a regmap to communicate with
 *
 * Return:	Spreading factor in chips / symbol
 */
u32
sx127X_get_lorasprf(struct regmap *map)
{
	u8 sf;
	u32 c_s;

	regmap_raw_read(map, SX127X_REG_MODEM_CONFIG2, &sf, 1);
	sf = sf >> 4;
	c_s = 1 << sf;

	return c_s;
}

const u32 hz[] = {
	  7800,
	 10400,
	 15600,
	 20800,
	 31250,
	 41700,
	 62500,
	125000,
	250000,
	500000
};

/**
 * sx127X_set_lorabw - Set RF bandwidth
 * @map:	the device as a regmap to communicate with
 * @bw:		RF bandwidth going to be assigned in Hz
 */
void
sx127X_set_lorabw(struct regmap *map, u32 bw)
{
	u8 i;
	u8 mcf1;

	for (i = 0; i < 9; i++) {
		if (hz[i] >= bw)
			break;
	}

	regmap_raw_read(map, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
	mcf1 = (mcf1 & 0x0F) | (i << 4);
	regmap_raw_write(map, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
}

/**
 * sx127X_get_lorabw - Get RF bandwidth
 * @map:	the device as a regmap to communicate with
 *
 * Return:	RF bandwidth in Hz
 */
u32
sx127X_get_lorabw(struct regmap *map)
{
	u8 mcf1;
	u8 bw;

	regmap_raw_read(map, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
	bw = mcf1 >> 4;

	return hz[bw];
}

/**
 * sx127X_set_loracr  - Set LoRa package's coding rate
 * @map:	the device as a regmap to communicate with
 * @cr:		Coding rate going to be assigned in a byte
 *		high 4 bits / low 4 bits: numerator / denominator
 */
void
sx127X_set_loracr(struct regmap *map, u8 cr)
{
	u8 mcf1;

	regmap_raw_read(map, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
	mcf1 = (mcf1 & 0x0E) | (((cr & 0xF) - 4) << 1);
	regmap_raw_write(map, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
}

/**
 * sx127X_get_loracr - Get LoRa package's coding rate
 * @map:	the device as a regmap to communicate with
 *
 * Return:	Coding rate in a byte
 *		high 4 bits / low 4 bits: numerator / denominator
 */
u8
sx127X_get_loracr(struct regmap *map)
{
	u8 mcf1;
	u8 cr;	/* ex: 0x45 represents cr=4/5 */

	regmap_raw_read(map, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
	cr = 0x40 + ((mcf1 & 0x0E) >> 1) + 4;

	return cr;
}

/**
 * sx127X_set_loraimplicit - Set LoRa packages with Explicit / Implicit Header
 * @map:	the device as a regmap to communicate with
 * @yesno:	1 / 0 for Implicit Header Mode / Explicit Header Mode
 */
void
sx127X_set_loraimplicit(struct regmap *map, u8 yesno)
{
	u8 mcf1;

	regmap_raw_read(map, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
	mcf1 = (yesno) ? (mcf1 | 0x01) : (mcf1 & 0xFE);
	regmap_raw_write(map, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
}

/**
 * sx127X_set_lorarxbytetimeout - Set RX operation time-out in terms of symbols
 * @map:	the device as a regmap to communicate with
 * @n:		Time-out in terms of symbols (bytes) going to be assigned
 */
void
sx127X_set_lorarxbytetimeout(struct regmap *map, u32 n)
{
	u8 buf[2];
	u8 mcf2;

	if (n < 1)
		n = 1;
	if (n > 1023)
		n = 1023;

	/* Read original Modem config 2. */
	regmap_raw_read(map, SX127X_REG_MODEM_CONFIG2, &mcf2, 1);

	/* LSB */
	buf[1] = n % 256;
	/* MSB */
	buf[0] = (mcf2 & 0xFC) | (n >> 8);

	regmap_raw_write(map, SX127X_REG_MODEM_CONFIG2, buf, 2);
}

/**
 * sx127X_set_lorarxtimeout - Set RX operation time-out seconds
 * @map:	the device as a regmap to communicate with
 * @ms:		The RX time-out time in ms
 */
void
sx127X_set_lorarxtimeout(struct regmap *map, u32 ms)
{
	u32 n;

	n = ms * sx127X_get_lorabw(map) / (sx127X_get_lorasprf(map) * 1000);

	sx127X_set_lorarxbytetimeout(map, n);
}

/**
 * sx127X_get_lorarxbytetimeout - Get RX operation time-out in terms of symbols
 * @map:	the device as a regmap to communicate with
 *
 * Return:	Time-out in terms of symbols (bytes)
 */
u32
sx127X_get_lorarxbytetimeout(struct regmap *map)
{
	u32 n;
	u8 buf[2];

	regmap_raw_read(map, SX127X_REG_MODEM_CONFIG2, buf, 2);

	n = (buf[0] & 0x03) * 256 + buf[1];

	return n;
}

/**
 * sx127X_get_lorarxtimeout - Get RX operation time-out seconds
 * @map:	the device as a regmap to communicate with
 *
 * Return:	The RX time-out time in ms
 */
u32
sx127X_get_lorarxtimeout(struct regmap *map)
{
	u32 ms;

	ms = 1000 * sx127X_get_lorarxbytetimeout(map) *
		sx127X_get_lorasprf(map) / sx127X_get_lorabw(map);

	return ms;
}

/**
 * sx127X_set_loramaxrxbuff - Maximum payload length in LoRa packet
 * @map:	the device as a regmap to communicate with
 * @len:	the max payload length going to be assigned in bytes
 */
void
sx127X_set_loramaxrxbuff(struct regmap *map, u8 len)
{
	regmap_raw_write(map, SX127X_REG_MAX_PAYLOAD_LENGTH, &len, 1);
}

/**
 * sx127X_get_loralastpktpayloadlen - Get the RX last packet payload length
 * @map:	the device as a regmap to communicate with
 *
 * Return:	the actual RX last packet payload length in bytes
 */
u8
sx127X_get_loralastpktpayloadlen(struct regmap *map)
{
	u8 len;

	regmap_raw_read(map, SX127X_REG_RX_NB_BYTES, &len, 1);

	return len;
}

/**
 * sx127X_readloradata - Read data from LoRa device (read RX FIFO)
 * @map:	the device as a regmap to communicate with
 * @buf:	buffer going to be read data into
 * @len:	the length of the data going to be read in bytes
 *
 * Return:	Positive / negtive values for the actual data length read from
 *		the LoRa device in bytes / failed
 */
ssize_t
sx127X_readloradata(struct regmap *map, u8 *buf, size_t len)
{
	u8 start_adr;
	int ret;

	/* Set chip FIFO pointer to FIFO last packet address. */
	start_adr = SX127X_FIFO_RX_BASE_ADDRESS;
	regmap_raw_write(map, SX127X_REG_FIFO_ADDR_PTR, &start_adr, 1);

	/* Read LoRa packet payload. */
	len = (len <= IEEE802154_MTU) ? len : IEEE802154_MTU;
	ret = regmap_raw_read(map, SX127X_REG_FIFO, buf, len);

	return (ret >= 0) ? len : ret;
}

/**
 * sx127X_sendloradata - Send data through LoRa device (write TX FIFO)
 * @rm:		the device as a regmap to communicate with
 * @buf:	buffer going to be send
 * @len:	the length of the buffer in bytes
 *
 * Return:	the actual length written into the LoRa device in bytes
 */
size_t
sx127X_sendloradata(struct regmap *map, u8 *buf, size_t len)
{
	u8 base_adr;
	u8 blen;

	/* Set chip FIFO pointer to FIFO TX base. */
	base_adr = SX127X_FIFO_TX_BASE_ADDRESS;
	regmap_raw_write(map, SX127X_REG_FIFO_ADDR_PTR, &base_adr, 1);

	/* Write payload synchronously to fill the FIFO of the chip. */
	blen = (len <= IEEE802154_MTU) ? len : IEEE802154_MTU;
	regmap_raw_write(map, SX127X_REG_FIFO, buf, blen);

	/* Set the FIFO payload length. */
	regmap_raw_write(map, SX127X_REG_PAYLOAD_LENGTH, &blen, 1);

	return blen;
}

/**
 * sx127X_get_loralastpktsnr - Get last LoRa packet's SNR
 * @map:	the device as a regmap to communicate with
 *
 * Return:	the last LoRa packet's SNR in db
 */
s32
sx127X_get_loralastpktsnr(struct regmap *map)
{
	s32 db;
	s8 snr;

	regmap_raw_read(map, SX127X_REG_PKT_SNR_VALUE, &snr, 1);
	db = snr / 4;

	return db;
}

/**
 * sx127X_get_loralastpktrssi - Get last LoRa packet's SNR
 * @map:	the device as a regmap to communicate with
 *
 * Return:	the last LoRa packet's RSSI in dbm
 */
s32
sx127X_get_loralastpktrssi(struct regmap *map)
{
	s32 dbm;
	u8 lhf;
	u8 rssi;
	s8 snr;

	/* Get LoRa is in high or low frequency mode. */
	lhf = sx127X_get_mode(map) & 0x08;
	/* Get RSSI value. */
	regmap_raw_read(map, SX127X_REG_PKT_RSSI_VALUE, &rssi, 1);
	dbm = (lhf) ? -164 + rssi : -157 + rssi;

	/* Adjust to correct the last packet RSSI if SNR < 0. */
	regmap_raw_read(map, SX127X_REG_PKT_SNR_VALUE, &snr, 1);
	if (snr < 0)
		dbm += snr / 4;

	return dbm;
}

/**
 * sx127X_get_lorarssi - Get current RSSI value
 * @map:	the device as a regmap to communicate with
 *
 * Return:	the current RSSI in dbm
 */
s32
sx127X_get_lorarssi(struct regmap *map)
{
	s32 dbm;
	u8 lhf;
	u8 rssi;

	/* Get LoRa is in high or low frequency mode. */
	lhf = sx127X_get_mode(map) & 0x08;
	/* Get RSSI value. */
	regmap_raw_read(map, SX127X_REG_RSSI_VALUE, &rssi, 1);
	dbm = (lhf) ? -164 + rssi : -157 + rssi;

	return dbm;
}

/**
 * sx127X_set_lorapreamblelen - Set LoRa preamble length
 * @map:	the device as a regmap to communicate with
 * @len:	the preamble length going to be assigned
 */
void
sx127X_set_lorapreamblelen(struct regmap *map, u32 len)
{
	u8 pl[2];

	pl[1] = len % 256;
	pl[0] = (len >> 8) % 256;

	regmap_raw_write(map, SX127X_REG_PREAMBLE_MSB, pl, 2);
}

/**
 * sx127X_get_lorapreamblelen - Get LoRa preamble length
 * @map:	the device as a regmap to communicate with
 *
 * Return:	length of the LoRa preamble
 */
u32
sx127X_get_lorapreamblelen(struct regmap *map)
{
	u8 pl[2];
	u32 len;

	regmap_raw_read(map, SX127X_REG_PREAMBLE_MSB, pl, 2);
	len = pl[0] * 256 + pl[1];

	return len;
}

/**
 * sx127X_set_loracrc - Enable CRC generation and check on received payload
 * @map:	the device as a regmap to communicate with
 * @yesno:	1 / 0 for check / not check
 */
void
sx127X_set_loracrc(struct regmap *map, u8 yesno)
{
	u8 mcf2;

	regmap_raw_read(map, SX127X_REG_MODEM_CONFIG2, &mcf2, 1);
	mcf2 = (yesno) ? mcf2 | (1 << 2) : mcf2 & (~(1 << 2));
	regmap_raw_write(map, SX127X_REG_MODEM_CONFIG2, &mcf2, 1);
}

/**
 * sx127X_set_boost - Set RF power amplifier boost in normal output range
 * @map:	the device as a regmap to communicate with
 * @yesno:	1 / 0 for boost / not boost
 */
void
sx127X_set_boost(struct regmap *map, u8 yesno)
{
	u8 pacf;

	regmap_raw_read(map, SX127X_REG_PA_CONFIG, &pacf, 1);
	pacf = (yesno) ? pacf | (1 << 7) : pacf & (~(1 << 7));
	regmap_raw_write(map, SX127X_REG_PA_CONFIG, &pacf, 1);
}

/**
 * sx127X_start_loramode - Start the device and set it in LoRa mode
 * @map:	the device as a regmap to communicate with
 */
void
sx127X_start_loramode(struct regmap *map)
{
	u8 op_mode;
	u8 base_adr;
#ifdef CONFIG_OF
	struct device_node *of_node = (regmap_get_device(map))->of_node;
#endif

	/* Get original OP Mode register. */
	op_mode = sx127X_get_mode(map);
	dev_dbg(regmap_get_device(map),
		"the original OP mode is 0x%X\n", op_mode);

	/* Set device to sleep state. */
	sx127X_set_state(map, SX127X_SLEEP_MODE);
	/* Set device to LoRa mode. */
	op_mode = sx127X_get_mode(map);
	op_mode = op_mode | 0x80;
	regmap_raw_write(map, SX127X_REG_OP_MODE, &op_mode, 1);
	/* Set device to standby state. */
	sx127X_set_state(map, SX127X_STANDBY_MODE);
	op_mode = sx127X_get_mode(map);
	dev_dbg(regmap_get_device(map),
		"the current OP mode is 0x%X\n", op_mode);

	/* Set LoRa in explicit header mode. */
	sx127X_set_loraimplicit(map, 0);

	/* Set chip FIFO RX base. */
	base_adr = SX127X_FIFO_RX_BASE_ADDRESS;
	regmap_raw_write(map, SX127X_REG_FIFO_RX_BASE_ADDR, &base_adr, 1);
	/* Set chip FIFO TX base. */
	base_adr = SX127X_FIFO_TX_BASE_ADDRESS;
	regmap_raw_write(map, SX127X_REG_FIFO_TX_BASE_ADDR, &base_adr, 1);

	/* Set the CSS spreading factor. */
#ifdef CONFIG_OF
	of_property_read_u32(of_node, "spreading-factor", &sprf);
#endif
	sx127X_set_lorasprf(map, sprf);

	/* Set RX time-out value. */
	sx127X_set_lorarxbytetimeout(map, rx_timeout);

	/* Clear all of the IRQ flags. */
	sx127X_clear_loraallflag(map);
	/* Set chip to RX state waiting for receiving. */
	sx127X_set_state(map, SX127X_RXSINGLE_MODE);
}

/**
 * init_sx127x - Initial the SX127X device
 * @map:	the device as a regmap to communicate with
 *
 * Return:	0 / negtive values for success / failed
 */
int
init_sx127x(struct regmap *map)
{
	int v;
#ifdef DEBUG
	u8 fv, mv;
#endif

	dev_dbg(regmap_get_device(map), "init sx127X\n");

	v = sx127X_read_version(map);
	if (v > 0) {
#ifdef DEBUG
		fv = (v >> 4) & 0xF;
		mv = v & 0xF;
		dev_dbg(regmap_get_device(map), "chip version %d.%d\n", fv, mv);
#endif
		return 0;
	} else {
		return -ENODEV;
	}
}

/*---------------------- SX1278 IEEE 802.15.4 Functions ----------------------*/

/* LoRa device's sensitivity in dbm. */
#ifndef SX1278_IEEE_SENSITIVITY
#define SX1278_IEEE_SENSITIVITY	(-148)
#endif
static s32 sensitivity = SX1278_IEEE_SENSITIVITY;
module_param(sensitivity, int, 0000);
MODULE_PARM_DESC(sensitivity, "RF receiver's sensitivity");

#define SX1278_IEEE_ENERGY_RANGE	(-sensitivity)

static int
sx1278_ieee_ed(struct ieee802154_hw *hw, u8 *level)
{
	struct sx1278_phy *phy = hw->priv;
	s32 rssi;
	s32 range = SX1278_IEEE_ENERGY_RANGE - 10;

	dev_dbg(regmap_get_device(phy->map), "%s\n", __func__);

	/* ED: IEEE  802.15.4-2011 8.2.5 Recevier ED. */
	rssi = sx127X_get_lorarssi(phy->map);
	if (rssi < (sensitivity + 10))
		*level = 0;
	else if (rssi >= 0)
		*level = 255;
	else
		*level = ((s32)255 * (rssi + range) / range) % 255;

	return 0;
}

#ifndef SX1278_IEEE_CHANNEL_MIN
#define SX1278_IEEE_CHANNEL_MIN		11
#endif
static u8 channel_min = SX1278_IEEE_CHANNEL_MIN;
module_param(channel_min, byte, 0000);
MODULE_PARM_DESC(channel_min, "Minimal channel number");

#ifndef SX1278_IEEE_CHANNEL_MAX
#define SX1278_IEEE_CHANNEL_MAX		11
#endif
static u8 channel_max = SX1278_IEEE_CHANNEL_MAX;
module_param(channel_max, byte, 0000);
MODULE_PARM_DESC(channel_max, "Maximum channel number");

#ifndef SX1278_IEEE_CENTER_CARRIER_FRQ
#define SX1278_IEEE_CENTER_CARRIER_FRQ	434000000
#endif
static u32 carrier_frq = SX1278_IEEE_CENTER_CARRIER_FRQ;
module_param(carrier_frq, uint, 0000);
MODULE_PARM_DESC(carrier_frq, "Center carrier frequency in Hz");

#ifndef SX1278_IEEE_BANDWIDTH
#define SX1278_IEEE_BANDWIDTH		500000
#endif
static u32 bandwidth = SX1278_IEEE_BANDWIDTH;
module_param(bandwidth, uint, 0000);
MODULE_PARM_DESC(bandwidth, "Bandwidth in Hz");

struct rf_frq {
	u32 carrier;
	u32 bw;
	u8 ch_min;
	u8 ch_max;
};

void
sx1278_ieee_get_rf_config(struct ieee802154_hw *hw, struct rf_frq *rf)
{
#ifdef CONFIG_OF
	struct sx1278_phy *phy = hw->priv;
	struct device_node *of_node = (regmap_get_device(phy->map))->of_node;

	/* Set the LoRa chip's center carrier frequency. */
	if (of_property_read_u32(of_node, "center-carrier-frq", &rf->carrier))
		rf->carrier = carrier_frq;

	/* Set the LoRa chip's RF bandwidth. */
	if (of_property_read_u32(of_node, "rf-bandwidth", &rf->bw))
		rf->bw = bandwidth;

	/* Set the LoRa chip's min & max RF channel if OF is defined. */
	if (of_property_read_u8(of_node, "minimal-RF-channel", &rf->ch_min))
		rf->ch_min = channel_min;

	if (of_property_read_u8(of_node, "maximum-RF-channel", &rf->ch_max))
		rf->ch_max = channel_max;
#else
	rf->carrier = carrier_frq;
	rf->bw = bandwidth;
	rf->ch_min = channel_min;
	rf->ch_max = channel_max;
#endif
}

static int
sx1278_ieee_set_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	struct sx1278_phy *phy = hw->priv;
	struct rf_frq rf;
	u32 fr;
	s8 d;

	dev_dbg(regmap_get_device(phy->map),
		"%s channel: %u\n", __func__, channel);

	sx1278_ieee_get_rf_config(hw, &rf);

	if (channel < rf.ch_min)
		channel = rf.ch_min;
	else if (channel > rf.ch_max)
		channel = rf.ch_max;

	d = channel - (rf.ch_min + rf.ch_max) / 2;
	fr = rf.carrier + d * rf.bw;

	sx127X_set_lorafrq(phy->map, fr);
	phy->opmode = sx127X_get_mode(phy->map);

	return 0;
}

/* in mbm */
s32 sx1278_powers[] = {
	-200, -100, 0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100,
	1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300};

static int
sx1278_ieee_set_txpower(struct ieee802154_hw *hw, s32 mbm)
{
	struct sx1278_phy *phy = hw->priv;
	s32 dbm = sx127X_mbm2dbm(mbm);

	dev_dbg(regmap_get_device(phy->map),
		"%s TX power: %d mbm\n", __func__, mbm);

	sx127X_set_lorapower(phy->map, dbm);

	return 0;
}

int
sx1278_ieee_rx(struct ieee802154_hw *hw)
{
	struct sx1278_phy *phy = hw->priv;
	bool do_rx;
	unsigned long f;

	dev_dbg(regmap_get_device(phy->map), "%s\n", __func__);

	spin_lock_irqsave(&phy->buf_lock, f);
	if (!phy->is_busy) {
		phy->is_busy = true;
		do_rx = true;
	} else {
		do_rx = false;
	}
	spin_unlock_irqrestore(&phy->buf_lock, f);

	if (do_rx) {
		sx127X_set_state(phy->map, SX127X_RXSINGLE_MODE);
		return 0;
	} else {
		dev_dbg(regmap_get_device(phy->map),
			"%s: device is busy\n", __func__);
		return -EBUSY;
	}
}

static int
sx1278_ieee_rx_complete(struct ieee802154_hw *hw)
{
	struct sx1278_phy *phy = hw->priv;
	struct sk_buff *skb;
	u8 len;
	u8 lqi;
	s32 rssi;
	s32 range = SX1278_IEEE_ENERGY_RANGE;
	int err;
	unsigned long f;

	skb = dev_alloc_skb(IEEE802154_MTU);
	if (!skb) {
		err = -ENOMEM;
		dev_err(regmap_get_device(phy->map),
			"%s: driver is out of memory\n", __func__);
		goto sx1278_ieee_rx_err;
	}

	len = sx127X_get_loralastpktpayloadlen(phy->map);
	sx127X_readloradata(phy->map, skb_put(skb, len), len);

	/* LQI: IEEE  802.15.4-2011 8.2.6 Link quality indicator. */
	rssi = sx127X_get_loralastpktrssi(phy->map);
	rssi = (rssi > 0) ? 0 : rssi;
	lqi = ((s32)255 * (rssi + range) / range) % 255;

	ieee802154_rx_irqsafe(hw, skb, lqi);

	dev_dbg(regmap_get_device(phy->map),
		"%s: len=%u LQI=%u\n", __func__, len, lqi);

	err = 0;

sx1278_ieee_rx_err:
	spin_lock_irqsave(&phy->buf_lock, f);
	phy->is_busy = false;
	spin_unlock_irqrestore(&phy->buf_lock, f);
	return err;
}

int
sx1278_ieee_tx(struct ieee802154_hw *hw)
{
	struct sx1278_phy *phy = hw->priv;
	struct sk_buff *tx_buf = phy->tx_buf;
	bool do_tx = false;
	unsigned long f;

	dev_dbg(regmap_get_device(phy->map),
		"%s: len=%u\n", __func__, tx_buf->len);

	if (!phy->post_tx_done) {
		sx127X_sendloradata(phy->map, tx_buf->data, tx_buf->len);
		phy->post_tx_done = true;
	}

	spin_lock_irqsave(&phy->buf_lock, f);
	if (!phy->is_busy) {
		phy->is_busy = true;
		do_tx = true;
		phy->one_to_be_sent = false;
	}
	spin_unlock_irqrestore(&phy->buf_lock, f);

	if (do_tx) {
		/* Set chip as TX state and transfer the data in FIFO. */
		phy->opmode = (phy->opmode & 0xF8) | SX127X_TX_MODE;
		regmap_write_async(phy->map, SX127X_REG_OP_MODE, phy->opmode);
		return 0;
	} else {
		dev_dbg(regmap_get_device(phy->map),
			"%s: device is busy\n", __func__);
		return -EBUSY;
	}
}

static int
sx1278_ieee_tx_complete(struct ieee802154_hw *hw)
{
	struct sx1278_phy *phy = hw->priv;
	struct sk_buff *skb = phy->tx_buf;
	unsigned long f;

	dev_dbg(regmap_get_device(phy->map), "%s\n", __func__);

	ieee802154_xmit_complete(hw, skb, false);

	spin_lock_irqsave(&phy->buf_lock, f);
	phy->is_busy = false;
	phy->tx_buf = NULL;
	spin_unlock_irqrestore(&phy->buf_lock, f);

	return 0;
}

static int
sx1278_ieee_xmit(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	struct sx1278_phy *phy = hw->priv;
	int ret;
	unsigned long f;

	dev_dbg(regmap_get_device(phy->map), "%s\n", __func__);

	WARN_ON(phy->suspended);

	spin_lock_irqsave(&phy->buf_lock, f);
	if (phy->tx_buf) {
		ret = -EBUSY;
	} else {
		phy->tx_buf = skb;
		phy->one_to_be_sent = true;
		phy->post_tx_done = false;
		ret = 0;
	}
	spin_unlock_irqrestore(&phy->buf_lock, f);

	return ret;
}

static int
sx1278_ieee_start(struct ieee802154_hw *hw)
{
	struct sx1278_phy *phy = hw->priv;

	dev_dbg(regmap_get_device(phy->map), "interface up\n");

	sx1278_ieee_set_channel(hw, 0, hw->phy->current_channel);
	phy->suspended = false;
	sx127X_start_loramode(phy->map);
	phy->opmode = sx127X_get_mode(phy->map);
	add_timer(&phy->timer);

	return 0;
}

static void
sx1278_ieee_stop(struct ieee802154_hw *hw)
{
	struct sx1278_phy *phy = hw->priv;

	dev_dbg(regmap_get_device(phy->map), "interface down\n");

	phy->suspended = true;
	del_timer(&phy->timer);
	sx127X_set_state(phy->map, SX127X_SLEEP_MODE);
}

static int
sx1278_ieee_set_promiscuous_mode(struct ieee802154_hw *hw, const bool on)
{
	return 0;
}

void
sx1278_ieee_statemachine(struct ieee802154_hw *hw)
{
	struct sx1278_phy *phy = hw->priv;
	u8 flags;
	u8 state;
	bool do_next_rx = false;
	unsigned long f;

	flags = sx127X_get_loraallflag(phy->map);
	state = sx127X_get_state(phy->map);

	if (flags & (SX127X_FLAG_RXTIMEOUT | SX127X_FLAG_PAYLOADCRCERROR)) {
		sx127X_clear_loraflag(phy->map, SX127X_FLAG_RXTIMEOUT
						| SX127X_FLAG_PAYLOADCRCERROR
						| SX127X_FLAG_RXDONE);
		spin_lock_irqsave(&phy->buf_lock, f);
		phy->is_busy = false;
		spin_unlock_irqrestore(&phy->buf_lock, f);
		do_next_rx = true;
	} else if (flags & SX127X_FLAG_RXDONE) {
		sx1278_ieee_rx_complete(phy->hw);
		sx127X_clear_loraflag(phy->map, SX127X_FLAG_RXDONE);
		do_next_rx = true;
	}

	if (flags & SX127X_FLAG_TXDONE) {
		sx1278_ieee_tx_complete(phy->hw);
		sx127X_clear_loraflag(phy->map, SX127X_FLAG_TXDONE);
		phy->tx_delay = 10;
		do_next_rx = true;
	}

	if (phy->one_to_be_sent &&
	    (state == SX127X_STANDBY_MODE) &&
	    (phy->tx_delay == 0)) {
		if (!sx1278_ieee_tx(phy->hw))
			do_next_rx = false;
	}

	if (do_next_rx)
		sx1278_ieee_rx(phy->hw);

	if (phy->tx_delay > 0)
		phy->tx_delay -= 1;

	if (!phy->suspended) {
		phy->timer.expires = jiffies_64 + 1;
		add_timer(&phy->timer);
	}
}

/**
 * sx1278_timer_irqwork - The actual work which checks the IRQ flags of the chip
 * @work:	the work entry listed in the workqueue
 */
static void
sx1278_timer_irqwork(struct work_struct *work)
{
	struct sx1278_phy *phy;

	phy = container_of(work, struct sx1278_phy, irqwork);
	sx1278_ieee_statemachine(phy->hw);
}

/**
 * sx1278_timer_isr - Callback function for the timer interrupt
 * @arg:	the general argument for this callback function
 */
static void
sx1278_timer_isr(struct timer_list *timer)
{
	struct sx1278_phy *phy = container_of(timer, struct sx1278_phy, timer);

	schedule_work(&phy->irqwork);
}

static const struct ieee802154_ops sx1278_ops = {
	.owner = THIS_MODULE,
	.xmit_async = sx1278_ieee_xmit,
	.ed = sx1278_ieee_ed,
	.set_channel = sx1278_ieee_set_channel,
	.set_txpower = sx1278_ieee_set_txpower,
	.start = sx1278_ieee_start,
	.stop = sx1278_ieee_stop,
	.set_promiscuous_mode = sx1278_ieee_set_promiscuous_mode,
};

/**
 * sx1278X_ieee_channel_mask - Get the available channels' mask of LoRa device
 * @hw:		LoRa IEEE 802.15.4 device
 *
 * Return:	The bitwise channel mask in 4 bytes
 */
u32
sx1278_ieee_channel_mask(struct ieee802154_hw *hw)
{
	struct rf_frq rf;
	u32 mask;

	sx1278_ieee_get_rf_config(hw, &rf);

	mask = ((u32)(1 << (rf.ch_max + 1)) - (u32)(1 << rf.ch_min));

	return mask;
}

static int
sx1278_ieee_add_one(struct sx1278_phy *phy)
{
	struct ieee802154_hw *hw = phy->hw;
	int err;

	/* Define channels could be used. */
	hw->phy->supported.channels[0] = sx1278_ieee_channel_mask(hw);
	/* SX1278 phy channel 11 as default */
	hw->phy->current_channel = 11;

	/* Define RF power. */
	hw->phy->supported.tx_powers = sx1278_powers;
	hw->phy->supported.tx_powers_size = ARRAY_SIZE(sx1278_powers);
	hw->phy->transmit_power = sx1278_powers[12];

	ieee802154_random_extended_addr(&hw->phy->perm_extended_addr);
	hw->flags = IEEE802154_HW_TX_OMIT_CKSUM
			| IEEE802154_HW_RX_OMIT_CKSUM
			| IEEE802154_HW_PROMISCUOUS;

	err = ieee802154_register_hw(hw);
	if (err)
		goto err_reg;

	INIT_WORK(&phy->irqwork, sx1278_timer_irqwork);

	timer_setup(&phy->timer, sx1278_timer_isr, 0);
	phy->timer.expires = jiffies_64 + HZ;

	spin_lock_init(&phy->buf_lock);

	err = init_sx127x(phy->map);
	if (err)
		goto err_reg;

	return 0;

err_reg:
	dev_err(regmap_get_device(phy->map),
		"register as IEEE 802.15.4 device failed\n");
	return err;
}

static void
sx1278_ieee_del(struct sx1278_phy *phy)
{
	if (!phy)
		return;

	del_timer(&phy->timer);
	flush_work(&phy->irqwork);

	ieee802154_unregister_hw(phy->hw);
	ieee802154_free_hw(phy->hw);
}

/*--------------------------- SX1278 SPI Functions ---------------------------*/

/* The compatible chip array. */
#ifdef CONFIG_OF
static const struct of_device_id sx1278_dt_ids[] = {
	{ .compatible = "semtech,sx1276" },
	{ .compatible = "semtech,sx1277" },
	{ .compatible = "semtech,sx1278" },
	{ .compatible = "semtech,sx1279" },
	{ .compatible = "sx1278" },
	{},
};
MODULE_DEVICE_TABLE(of, sx1278_dt_ids);
#endif

/* The compatible ACPI device array. */
#ifdef CONFIG_ACPI
static const struct acpi_device_id sx1278_acpi_ids[] = {
	{ .id = "sx1278" },
	{},
};
MODULE_DEVICE_TABLE(acpi, sx1278_acpi_ids);
#endif

/* The compatible SPI device id array. */
static const struct spi_device_id sx1278_spi_ids[] = {
	{ .name = "sx1278" },
	{},
};
MODULE_DEVICE_TABLE(spi, sx1278_spi_ids);

bool sx1278_reg_volatile(struct device *dev, unsigned int reg)
{
	return true;
}

/* The SX1278 regmap config. */
struct regmap_config sx1278_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = SX127X_MAX_REG,
	.read_flag_mask = 0x00,
	.write_flag_mask = 0x80,
	.volatile_reg = sx1278_reg_volatile,
};

/* The SPI probe callback function. */
static int sx1278_spi_probe(struct spi_device *spi)
{
	struct ieee802154_hw *hw;
	struct sx1278_phy *phy;
	int err;

	hw = ieee802154_alloc_hw(sizeof(*phy), &sx1278_ops);
	if (!hw) {
		dev_err(&spi->dev, "not enough memory\n");
		return -ENOMEM;
	}

	phy = hw->priv;
	phy->hw = hw;
	hw->parent = &spi->dev;
	phy->map = devm_regmap_init_spi(spi, &sx1278_regmap_config);

	/* Set the SPI device's driver data for later usage. */
	spi_set_drvdata(spi, phy);

	err = sx1278_ieee_add_one(phy);
	if (err < 0) {
		dev_err(&spi->dev, "no SX1278 compatible device\n");
		goto sx1278_spi_probe_err;
	}

	dev_info(&spi->dev,
		 "add an IEEE 802.15.4 over LoRa SX1278 compatible device\n");

	return 0;

sx1278_spi_probe_err:
	sx1278_ieee_del(phy);
	return err;
}

/* The SPI remove callback function. */
static int sx1278_spi_remove(struct spi_device *spi)
{
	struct sx1278_phy *phy = spi_get_drvdata(spi);

	sx1278_ieee_del(phy);

	return 0;
}

#define __DRIVER_NAME	"sx1278"

/* The SPI driver which acts as a protocol driver in this kernel module. */
static struct spi_driver sx1278_spi_driver = {
	.driver = {
		.name = __DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(sx1278_dt_ids),
#endif
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(sx1278_acpi_ids),
#endif
	},
	.probe = sx1278_spi_probe,
	.remove = sx1278_spi_remove,
	.id_table = sx1278_spi_ids,
};

/* Register SX1278 kernel module. */
module_spi_driver(sx1278_spi_driver);

MODULE_AUTHOR("Jian-Hong Pan, <starnight@g.ncu.edu.tw>");
MODULE_DESCRIPTION("LoRa device SX1278 driver with IEEE 802.15.4 interface");
MODULE_LICENSE("Dual BSD/GPL");
