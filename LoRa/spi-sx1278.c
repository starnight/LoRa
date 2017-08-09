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
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/compat.h>
#include <linux/acpi.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/skbuff.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <asm/div64.h>

#include "lora.h"

/*------------------------------ LoRa Functions ------------------------------*/

#ifndef F_XOSC
#define F_XOSC		32000000
#endif
#define	__POW_2_19	0x80000

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

/**
 * sx127X_readVersion - Get LoRa device's chip version
 * @rm:		the device as a regmap to communicate with
 * @vstr:	the buffer going to hold the chip's version string
 * @len:	the max length of the buffer
 *
 * Return:	Positive / negtive values for version code / failed
 * 		Version code:	bits 7-4 full version number,
 * 				bits 3-0 metal mask revision number
 */
int
sx127X_readVersion(struct regmap *rm)
{
	uint8_t v;
	int status;

	status = regmap_raw_read(rm, SX127X_REG_VERSION, &v, 1);

	if ((status == 0) && (0 < v) && (v < 0xFF))
		status = v;
	else
		status = -ENODEV;

	return status;
}

/**
 * sx127X_getMode - Get LoRa device's mode register
 * @rm:		the device as a regmap to communicate with
 *
 * Return:	LoRa device's register value
 */
uint8_t 
sx127X_getMode(struct regmap *rm)
{
	uint8_t op_mode;

	/* Get original OP Mode register. */
	regmap_raw_read(rm, SX127X_REG_OP_MODE, &op_mode, 1);

	return op_mode;
}

/**
 * sx127X_getState - Set LoRa device's operating state
 * @rm:		the device as a regmap to communicate with
 * @st:		LoRa device's operating state going to be assigned
 */
void
sx127X_setState(struct regmap *rm, uint8_t st)
{
	uint8_t op_mode;

	/* Get original OP Mode register. */
	op_mode = sx127X_getMode(rm);
	/* Set device to designated state. */
	op_mode = (op_mode & 0xF8) | (st & 0x07);
	regmap_raw_write(rm, SX127X_REG_OP_MODE, &op_mode, 1);
}

/**
 * sx127X_getState - Get LoRa device's operating state
 * @rm:		the device as a regmap to communicate with
 *
 * Return:	LoRa device's operating state
 */
uint8_t
sx127X_getState(struct regmap *rm)
{
	uint8_t op_mode;

	op_mode = sx127X_getMode(rm) & 0x07;

	return op_mode;
}

/**
 * sx127X_getLoRaFreq - Set RF frequency
 * @rm:		the device as a regmap to communicate with
 * @fr:		RF frequency going to be assigned in Hz
 */
void
sx127X_setLoRaFreq(struct regmap *rm, uint32_t fr)
{
	uint64_t frt64;
	uint32_t frt;
	uint8_t buf[3];
	int i;
	uint32_t f_xosc;

#ifdef CONFIG_OF
	/* Set the LoRa module's crystal oscillator's clock if OF is defined. */
	const void *ptr;

	ptr = of_get_property((regmap_get_device(rm))->of_node,
			"clock-frequency",
			NULL);
	f_xosc = (ptr != NULL) ? be32_to_cpup(ptr) : F_XOSC;
#else
	f_xosc = F_XOSC;
#endif

	frt64 = (uint64_t)fr * (uint64_t)__POW_2_19;
	do_div(frt64, f_xosc);
	frt = frt64;

	for (i = 2; i >= 0; i--) {
		buf[i] = frt % 256;
		frt = frt >> 8;
	}

	regmap_raw_write(rm, SX127X_REG_FRF_MSB, buf, 3);
}

/**
 * sx127X_getLoRaFreq - Get RF frequency
 * @rm:		the device as a regmap to communicate with
 *
 * Return:	RF frequency in Hz
 */
uint32_t
sx127X_getLoRaFreq(struct regmap *rm)
{
	uint64_t frt = 0;
	uint8_t buf[3];
	int status;
	int i;
	uint32_t fr;
	uint32_t f_xosc;

#ifdef CONFIG_OF
	/* Set the LoRa module's crystal oscillator's clock if OF is defined. */
	const void *ptr;

	ptr = of_get_property((regmap_get_device(rm))->of_node,
			"clock-frequency",
			NULL);
	f_xosc = (ptr != NULL) ? be32_to_cpup(ptr) : F_XOSC;
#else
	f_xosc = F_XOSC;
#endif

	status = regmap_raw_read(rm, SX127X_REG_FRF_MSB, buf, 3);
	if (status < 0)
		return 0.0;

	for (i = 0; i <= 2; i++)
		frt = frt * 256 + buf[i];

	fr =  frt * f_xosc / __POW_2_19;

	return fr;
}

/**
 * sx127X_setLoRaPower - Set RF output power
 * @rm:		the device as a regmap to communicate with
 * @pout:	RF output power going to be assigned in dbm
 */
void
sx127X_setLoRaPower(struct regmap *rm, int32_t pout)
{
	uint8_t pacf;
	uint8_t boost;
	uint8_t outputPower;
	int32_t pmax;

	if (pout > 14) {
		/* Pout > 14dbm */
		boost = 1;
		pmax = 7;
		outputPower = pout - 2;
	}
	else if (pout < 0) {
		/* Pout < 0dbm */
		boost = 0;
		pmax = 2;
		outputPower = 3 + pout;
	}
	else {
		/* 0dbm <= Pout <= 14dbm */
		boost = 0;
		pmax = 7;
		outputPower = pout;
	}

	pacf = (boost << 7) | (pmax << 4) | (outputPower);
	regmap_raw_write(rm, SX127X_REG_PA_CONFIG, &pacf, 1);
}

/**
 * sx127X_getLoRaPower - Get RF output power
 * @rm:		the device as a regmap to communicate with
 *
 * Return:	RF output power in dbm
 */
int32_t
sx127X_getLoRaPower(struct regmap *rm)
{
	uint8_t pac;
	uint8_t boost;
	int32_t outputPower;
	int32_t pmax;
	int32_t pout;

	regmap_raw_read(rm, SX127X_REG_PA_CONFIG, &pac, 1);
	boost = (pac & 0x80) >> 7;
	outputPower = pac & 0x0F;
	if (boost) {
		pout = 2 + outputPower;
	}
	else {
		/* Power max should be pmax/10.  It is 10 times for now. */
		pmax = (108 + 6 * ((pac & 0x70) >> 4));
		pout = (pmax - (150 - outputPower * 10)) / 10;
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

int8_t lna_gain[] = {
	 0,
	-6,
	-12,
	-24,
	-26,
	-48
};

/**
 * sx127X_setLoRaLNA - Set RF LNA gain
 * @rm:		the device as a regmap to communicate with
 * @db:		RF LNA gain going to be assigned in db
 */
void
sx127X_setLoRaLNA(struct regmap *rm, int32_t db)
{
	uint8_t i, g;
	uint8_t lnacf;

	for (i = 0; i < 5; i++) {
		if (lna_gain[i] <= db)
			break;
	}
	g = i + 1;

	regmap_raw_read(rm, SX127X_REG_LNA, &lnacf, 1);
	lnacf = (lnacf & 0x1F) | (g << 5);
	regmap_raw_write(rm, SX127X_REG_LNA, &lnacf, 1);
}

/**
 * sx127X_getLoRaLNA - Get RF LNA gain
 * @rm:		the device as a regmap to communicate with
 *
 * Return:	RF LNA gain db
 */
int32_t
sx127X_getLoRaLNA(struct regmap *rm)
{
	int32_t db;
	int8_t i, g;
	uint8_t lnacf;

	regmap_raw_read(rm, SX127X_REG_LNA, &lnacf, 1);
	g = (lnacf >> 5);
	i = g - 1;
	db = lna_gain[i];

	return db;
}

/**
 * sx127X_setLoRaLNAAGC - Set RF LNA go with auto gain control or manual
 * @rm:		the device as a regmap to communicate with
 * @yesno:	1 / 0 for auto gain control / manual
 */
void
sx127X_setLoRaLNAAGC(struct regmap *rm, int32_t yesno)
{
	uint8_t mcf3;

	regmap_raw_read(rm, SX127X_REG_MODEM_CONFIG3, &mcf3, 1);
	mcf3 = (yesno) ? (mcf3 | 0x04) : (mcf3 & (~0x04));
	regmap_raw_write(rm, SX127X_REG_MODEM_CONFIG3, &mcf3, 1);
}

/**
 * sx127X_getLoRaAllFlag - Get all of the LoRa device's IRQ flags' current state
 * @rm:		the device as a regmap to communicate with
 *
 * Return:	All of the LoRa device's IRQ flags' current state in a byte
 */
uint8_t
sx127X_getLoRaAllFlag(struct regmap *rm)
{
	uint8_t flags;

	regmap_raw_read(rm, SX127X_REG_IRQ_FLAGS, &flags, 1);

	return flags;
}

/**
 * sx127X_getLoRaAllFlag - Get interested LoRa device's IRQ flag's current state
 * @rm:		the device as a regmap to communicate with
 * @f:		the interested LoRa device's IRQ flag
 *
 * Return:	The interested LoRa device's IRQ flag's current state in a byte
 */
#define sx127X_getLoRaFlag(rm, f)	(sx127X_getLoRaAllFlag(rm) & (f))

/**
 * sx127X_clearLoRaFlag - Clear designated LoRa device's IRQ flag
 * @rm:		the device as a regmap to communicate with
 * @f:		flags going to be cleared
 */
void
sx127X_clearLoRaFlag(struct regmap *rm, uint8_t f)
{
	uint8_t flag;

	/* Get oiginal flag. */
	flag = sx127X_getLoRaAllFlag(rm);
	/* Set the designated bits of the flag. */
	flag |= f;
	regmap_raw_write(rm, SX127X_REG_IRQ_FLAGS, &flag, 1);
}

/**
 * sx127X_clearLoRaAllFlag - Clear designated LoRa device's all IRQ flags
 * @rm:		the device as a regmap to communicate with
 */
#define sx127X_clearLoRaAllFlag(spi)	sx127X_clearLoRaFlag(spi, 0xFF)

/**
 * sx127X_getLoRaSPRFactor - Get the RF modulation's spreading factor
 * @rm:		the device as a regmap to communicate with
 * @c_s:	Spreading factor in chips / symbol
 */
void
sx127X_setLoRaSPRFactor(struct regmap *rm, uint32_t c_s)
{
	uint8_t sf;
	uint8_t mcf2;

	for (sf = 6; sf < 12; sf++) {
		if (c_s == ((uint32_t)1 << sf))
			break;
	}

	regmap_raw_read(rm, SX127X_REG_MODEM_CONFIG2, &mcf2, 1);
	mcf2 = (mcf2 & 0x0F) | (sf << 4);
	regmap_raw_write(rm, SX127X_REG_MODEM_CONFIG2, &mcf2, 1);
}

/**
 * sx127X_getLoRaSPRFactor - Get the RF modulation's spreading factor
 * @rm:		the device as a regmap to communicate with
 *
 * Return:	Spreading factor in chips / symbol
 */
uint32_t
sx127X_getLoRaSPRFactor(struct regmap *rm)
{
	uint8_t sf;
	uint32_t c_s;

	regmap_raw_read(rm, SX127X_REG_MODEM_CONFIG2, &sf, 1);
	sf = sf >> 4;
	c_s = 1 << sf;

	return c_s;
}

const uint32_t hz[] = {
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
 * sx127X_getLoRaBW - Set RF bandwidth
 * @rm:		the device as a regmap to communicate with
 * @bw:		RF bandwidth going to be assigned in Hz
 */
void
sx127X_setLoRaBW(struct regmap *rm, uint32_t bw)
{
	uint8_t i;
	uint8_t mcf1;

	for (i = 0; i < 9; i++) {
		if (hz[i] >= bw)
			break;
	}

	regmap_raw_read(rm, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
	mcf1 = (mcf1 & 0x0F) | (i << 4);
	regmap_raw_write(rm, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
}

/**
 * sx127X_getLoRaBW - Get RF bandwidth
 * @rm:		the device as a regmap to communicate with
 *
 * Return:	RF bandwidth in Hz
 */
uint32_t
sx127X_getLoRaBW(struct regmap *rm)
{
	uint8_t mcf1;
	uint8_t bw;

	regmap_raw_read(rm, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
	bw = mcf1 >> 4;

	return hz[bw];
}

/**
 * sx127X_setLoRaCR  - Get LoRa package's coding rate
 * @rm:		the device as a regmap to communicate with
 * @cr:		Coding rate going to be assigned in a byte
 * 		high 4 bits / low 4 bits: numerator / denominator
 */
void
sx127X_setLoRaCR(struct regmap *rm, uint8_t cr)
{
	uint8_t mcf1;

	regmap_raw_read(rm, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
	mcf1 = (mcf1 & 0x0E) | (((cr & 0xF) - 4) << 1);
	regmap_raw_write(rm, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
}

/**
 * sx127X_getLoRaCR - Get LoRa package's coding rate
 * @rm:		the device as a regmap to communicate with
 *
 * Return:	Coding rate in a byte
 * 		high 4 bits / low 4 bits: numerator / denominator
 */
uint8_t
sx127X_getLoRaCR(struct regmap *rm)
{
	uint8_t mcf1;
	uint8_t cr;	/* ex: 0x45 represents cr=4/5 */

	regmap_raw_read(rm, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
	cr = 0x40 + ((mcf1 & 0x0E) >> 1) + 4;
	
	return cr;
}

/**
 * sx127X_setLoRaImplicit - Set LoRa packages in Explicit / Implicit Header Mode
 * @rm:		the device as a regmap to communicate with
 * @yesno:	1 / 0 for Implicit Header Mode / Explicit Header Mode
 */
void
sx127X_setLoRaImplicit(struct regmap *rm, uint8_t yesno)
{
	uint8_t mcf1;

	regmap_raw_read(rm, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
	mcf1 = (yesno) ? (mcf1 | 0x01) : (mcf1 & 0xFE);
	regmap_raw_write(rm, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
}

/**
 * sx127X_setLoRaRXByteTimeout - Get RX operation time-out in terms of symbols
 * @rm:		the device as a regmap to communicate with
 * @n:		Time-out in terms of symbols (bytes) going to be assigned
 */
void
sx127X_setLoRaRXByteTimeout(struct regmap *rm, uint32_t n)
{
	uint8_t buf[2];
	uint8_t mcf2;

	if (n < 1)
		n = 1;
	if (n > 1023)
		n = 1023;

	/* Read original Modem config 2. */
	regmap_raw_read(rm, SX127X_REG_MODEM_CONFIG2, &mcf2, 1);

	/* LSB */
	buf[1] = n % 256;
	/* MSB */
	buf[0] = (mcf2 & 0xFC) | (n >> 8);

	regmap_raw_write(rm, SX127X_REG_MODEM_CONFIG2, buf, 2);
}

/**
 * sx127X_setLoRaRXTimeout - Set RX operation time-out seconds
 * @rm:		the device as a regmap to communicate with
 * @ms:		The RX time-out time in ms
 */
void
sx127X_setLoRaRXTimeout(struct regmap *rm, uint32_t ms)
{
	uint32_t n;

	n = ms * sx127X_getLoRaBW(rm) / (sx127X_getLoRaSPRFactor(rm) * 1000);

	sx127X_setLoRaRXByteTimeout(rm, n);
}

/**
 * sx127X_getLoRaRXByteTimeout - Get RX operation time-out in terms of symbols
 * @rm:		the device as a regmap to communicate with
 *
 * Return:	Time-out in terms of symbols (bytes)
 */
uint32_t
sx127X_getLoRaRXByteTimeout(struct regmap *rm)
{
	uint32_t n;
	uint8_t buf[2];

	regmap_raw_read(rm, SX127X_REG_MODEM_CONFIG2, buf, 2);

	n = (buf[0] & 0x03) * 256 + buf[1];

	return n;
}

/**
 * sx127X_getLoRaRXTimeout - Get RX operation time-out seconds
 * @rm:		the device as a regmap to communicate with
 *
 * Return:	The RX time-out time in ms
 */
uint32_t
sx127X_getLoRaRXTimeout(struct regmap *rm)
{
	uint32_t ms;

	ms = 1000 * sx127X_getLoRaRXByteTimeout(rm) * \
		sx127X_getLoRaSPRFactor(rm) / sx127X_getLoRaBW(rm);

	return ms;
}

/**
 * sx127X_setLoRaMaxRXBuff - Maximum payload length in LoRa packet
 * @rm:		the device as a regmap to communicate with
 * @len:	the max payload length going to be assigned in bytes
 */
void
sx127X_setLoRaMaxRXBuff(struct regmap *rm, uint8_t len)
{
	regmap_raw_write(rm, SX127X_REG_MAX_PAYLOAD_LENGTH, &len, 1);
}

/**
 * sx127X_getLoRaLastPacketPayloadLen - Get the RX last packet payload length
 * @rm:		the device as a regmap to communicate with
 *
 * Return:	the actual RX last packet payload length in bytes
 */
uint8_t
sx127X_getLoRaLastPacketPayloadLen(struct regmap *rm)
{
	uint8_t len;

	regmap_raw_read(rm, SX127X_REG_RX_NB_BYTES, &len, 1);

	return len;
}

/**
 * sx127X_readLoRaData - Read data from LoRa device (RX)
 * @rm:		the device as a regmap to communicate with
 * @buf:	buffer going to be read data into
 * @len:	the length of the data going to be read in bytes
 *
 * Return:	Positive / negtive values for the actual data length read from
 * 		the LoRa device in bytes / failed
 */
ssize_t
sx127X_readLoRaData(struct regmap *rm, uint8_t *buf, size_t len)
{
	uint8_t start_adr;
	int ret;

	/* Get the chip RX FIFO last packet address. */
	regmap_raw_read(rm, SX127X_REG_FIFO_RX_CURRENT_ADDR, &start_adr, 1);
	/* Set chip FIFO pointer to FIFO last packet address. */
	regmap_raw_write(rm, SX127X_REG_FIFO_ADDR_PTR, &start_adr, 1);

	/* Read LoRa packet payload. */
	ret = regmap_raw_read(rm, SX127X_REG_FIFO, buf, len);

	dev_dbg(regmap_get_device(rm),
		"read %zu bytes from 0x%u with ret=%d\n", len, start_adr, ret);

	return (ret >= 0) ? len : ret;
}

/**
 * sx127X_sendLoRaData - Send data out through LoRa device (TX)
 * @rm:		the device as a regmap to communicate with
 * @buf:	buffer going to be send
 * @len:	the length of the buffer in bytes
 *
 * Return:	the actual length written into the LoRa device in bytes
 */
ssize_t
sx127X_sendLoRaData(struct regmap *rm, uint8_t *buf, size_t len)
{
	uint8_t base_adr;
	uint8_t blen;

	/* Set chip FIFO pointer to FIFO TX base. */
	regmap_raw_read(rm, SX127X_REG_FIFO_TX_BASE_ADDR, &base_adr, 1);
	regmap_raw_write(rm, SX127X_REG_FIFO_ADDR_PTR, &base_adr, 1);

#define SX127X_MAX_FIFO_LENGTH	0xFF
	blen = (len < SX127X_MAX_FIFO_LENGTH) ? len : SX127X_MAX_FIFO_LENGTH;

	/* Write to SPI chip synchronously to fill the FIFO of the chip. */
	regmap_raw_write(rm, SX127X_REG_FIFO, buf, blen);

	/* Set the FIFO payload length. */
	regmap_raw_write(rm, SX127X_REG_PAYLOAD_LENGTH, &blen, 1);

	return blen;
}

/**
 * sx127X_getLoRaLastPacketSNR - Get last LoRa packet's SNR
 * @rm:		the device as a regmap to communicate with
 *
 * Return:	the last LoRa packet's SNR in db
 */
int32_t
sx127X_getLoRaLastPacketSNR(struct regmap *rm)
{
	int32_t db;
	int8_t snr;

	regmap_raw_read(rm, SX127X_REG_PKT_SNR_VALUE, &snr, 1);
	db = snr / 4;

	return db;
}

/**
 * sx127X_getLoRaLastPacketRSSI - Get last LoRa packet's SNR
 * @rm:		the device as a regmap to communicate with
 *
 * Return:	the last LoRa packet's RSSI in dbm
 */
int32_t
sx127X_getLoRaLastPacketRSSI(struct regmap *rm)
{
	int32_t dbm;
	uint8_t lhf;
	uint8_t rssi;
	int8_t snr;

	/* Get LoRa is in high or low frequency mode. */
	lhf = sx127X_getMode(rm) & 0x08;
	/* Get RSSI value. */
	regmap_raw_read(rm, SX127X_REG_PKT_RSSI_VALUE, &rssi, 1);
	dbm = (lhf) ? -164 + rssi : -157 + rssi;

	/* Adjust to correct the last packet RSSI if SNR < 0. */
	regmap_raw_read(rm, SX127X_REG_PKT_SNR_VALUE, &snr, 1);
	if(snr < 0)
		dbm += snr / 4;

	return dbm;
}

/**
 * sx127X_getLoRaRSSI - Get current RSSI value
 * @rm:		the device as a regmap to communicate with
 *
 * Return:	the current RSSI in dbm
 */
int32_t
sx127X_getLoRaRSSI(struct regmap *rm)
{
	int32_t dbm;
	uint8_t lhf;
	uint8_t rssi;

	/* Get LoRa is in high or low frequency mode. */
	lhf = sx127X_getMode(rm) & 0x08;
	/* Get RSSI value. */
	regmap_raw_read(rm, SX127X_REG_RSSI_VALUE, &rssi, 1);
	dbm = (lhf) ? -164 + rssi : -157 + rssi;

	return dbm;
}

/**
 * sx127X_setLoRaPreambleLen - Set LoRa preamble length
 * @rm:		the device as a regmap to communicate with
 * @len:	the preamble length going to be assigned
 */
void
sx127X_setLoRaPreambleLen(struct regmap *rm, uint32_t len)
{
	uint8_t pl[2];

	pl[1] = len % 256;
	pl[0] = (len >> 8) % 256;

	regmap_raw_write(rm, SX127X_REG_PREAMBLE_MSB, pl, 2);
}

/**
 * sx127X_getLoRaPreambleLen - Get LoRa preamble length
 * @rm:		the device as a regmap to communicate with
 *
 * Return:	length of the LoRa preamble
 */
uint32_t
sx127X_getLoRaPreambleLen(struct regmap *rm)
{
	uint8_t pl[2];
	uint32_t len;

	regmap_raw_read(rm, SX127X_REG_PREAMBLE_MSB, pl, 2);
	len = pl[0] * 256 + pl[1];

	return len;
}

/**
 * sx127X_setLoRaCRC - Enable CRC generation and check on received payload
 * @rm:		the device as a regmap to communicate with
 * @yesno:	1 / 0 for check / not check
 */
void
sx127X_setLoRaCRC(struct regmap *rm, uint8_t yesno)
{
	uint8_t mcf2;

	regmap_raw_read(rm, SX127X_REG_MODEM_CONFIG2, &mcf2, 1);
	mcf2 = (yesno) ? mcf2 | (1 << 2) : mcf2 & (~(1 << 2));
	regmap_raw_write(rm, SX127X_REG_MODEM_CONFIG2, &mcf2, 1);
}

/**
 * sx127X_setBoost - Set RF power amplifier boost in normal output range
 * @rm:		the device as a regmap to communicate with
 * @yesno:	1 / 0 for boost / not boost
 */
void
sx127X_setBoost(struct regmap *rm, uint8_t yesno)
{
	uint8_t pacf;

	regmap_raw_read(rm, SX127X_REG_PA_CONFIG, &pacf, 1);
	pacf = (yesno) ? pacf | (1 << 7) : pacf & (~(1 << 7));
	regmap_raw_write(rm, SX127X_REG_PA_CONFIG, &pacf, 1);
}

/**
 * sx127X_startLoRaMode - Start the device and set it in LoRa mode
 * @rm:		the device as a regmap to communicate with
 */
void
sx127X_startLoRaMode(struct regmap *rm)
{
	uint8_t op_mode;
	uint8_t base_adr;

	/* Get original OP Mode register. */
	op_mode = sx127X_getMode(rm);
	dev_dbg(regmap_get_device(rm),
		"the original OP mode is 0x%X\n",
		op_mode);
	/* Set device to sleep state. */
	sx127X_setState(rm, SX127X_SLEEP_MODE);
	/* Set device to LoRa mode. */
	op_mode = sx127X_getMode(rm);
	op_mode = op_mode | 0x80;
	regmap_raw_write(rm, SX127X_REG_OP_MODE, &op_mode, 1);
	/* Set device to standby state. */
	sx127X_setState(rm, SX127X_STANDBY_MODE);
	op_mode = sx127X_getMode(rm);
	dev_dbg(regmap_get_device(rm),
		"the current OP mode is 0x%X\n",
		op_mode);

	/* Set LoRa in explicit header mode. */
	sx127X_setLoRaImplicit(rm, 0);

	/* Set chip FIFO RX base. */
	base_adr = 0x00;
	dev_dbg(regmap_get_device(rm), "going to set RX base address\n");
	regmap_raw_write(rm, SX127X_REG_FIFO_RX_BASE_ADDR, &base_adr, 1);
	/* Set chip FIFO TX base. */
	base_adr = 0x80;
	dev_dbg(regmap_get_device(rm), "going to set TX base address\n");
	regmap_raw_write(rm, SX127X_REG_FIFO_TX_BASE_ADDR, &base_adr, 1);

	/* Clear all of the IRQ flags. */
	sx127X_clearLoRaAllFlag(rm);
	/* Set chip to RX continuous state waiting for receiving. */
	sx127X_setState(rm, SX127X_RXCONTINUOUS_MODE);
}

/**
 * init_sx127X - Initial the SX127X device
 * @rm:		the device as a regmap to communicate with
 *
 * Return:	Positive / negtive values for version code / failed
 * 		Version code:	bits 7-4 full version number,
 * 				bits 3-0 metal mask revision number
 */
int
init_sx127X(struct regmap *rm)
{
	int v;
#ifdef DEBUG
	uint8_t fv, mmv;
#endif

	dev_dbg(regmap_get_device(rm), "init sx127X\n");

	v = sx127X_readVersion(rm);
	if (v > 0) {
#ifdef DEBUG
		fv = (v >> 4) & 0xF;
		mmv = v & 0xF;
		dev_dbg(regmap_get_device(rm), "chip version %d.%d\n", fv, mmv);
#endif
#ifdef LORA_DEBUG_FS
		sx127X_startLoRaMode(rm);
#endif
	}

	return v;
}

/*---------------------- SX127X IEEE 802.15.4 Functions ----------------------*/

/* LoRa device's sensitivity in dbm. */
#ifndef SX127X_IEEE_SENSITIVITY
#define SX127X_IEEE_SENSITIVITY	(-148)
#endif
#define SX127X_IEEE_ENERGY_RANGE	(-SX127X_IEEE_SENSITIVITY)

/**
 * sx127X_ieee_rx - Read the frame from LoRa device's RX
 * @lrdata:	LoRa device
 *
 * Return:	0 for success, negative number for error
 */
int
sx127X_ieee_rx(struct lora_struct *lrdata)
{
	struct regmap *rm = lrdata->lora_device;
	struct sk_buff *skb;
	uint8_t len;
	uint8_t lqi;
	int32_t rssi;
	int32_t range = SX127X_IEEE_ENERGY_RANGE;
	int ret;

	len = sx127X_getLoRaLastPacketPayloadLen(rm);
	if (!ieee802154_is_valid_psdu_len(len)) {
		dev_dbg(regmap_get_device(rm), "corrupted frame received\n");
		len = IEEE802154_MTU;
	}

	skb = dev_alloc_skb(len);
	if (!skb) {
		dev_err(regmap_get_device(rm), "%s out of memory\n", __func__);
		ret = -ENOMEM;
		goto sx127x_ieee_rx_ret;
	}

	ret = sx127X_readLoRaData(rm, skb_put(skb, len), len);
	if (ret <= 0) {
		dev_dbg(regmap_get_device(rm), "frame reception failed\n");
		kfree(skb);
		if (ret == 0)
			ret = -EINVAL;
		goto sx127x_ieee_rx_ret;
	}

	/* LQI: IEEE  802.15.4-2011 8.2.6 Link quality indicator. */
	rssi = sx127X_getLoRaLastPacketRSSI(rm);
	rssi = (rssi > 0) ? 0 : rssi;
	lqi = ((int32_t)255 * (rssi + range) / range) % 255;

	ieee802154_rx_irqsafe(lrdata->hw, skb, lqi);

	dev_dbg(regmap_get_device(rm),
		"%s: len=%u LQI=%u\n", __func__, len, lqi);

	ret = 0;

sx127x_ieee_rx_ret:
	mutex_lock(&(lrdata->buf_lock));
	if (lrdata->tx_skb == NULL) {
		sx127X_setState(rm, SX127X_STANDBY_MODE);
		sx127X_setState(rm, SX127X_RXCONTINUOUS_MODE);
	}
	mutex_unlock(&(lrdata->buf_lock));
	return ret;
}

void
sx127X_ieee_rx_error(struct lora_struct *lrdata)
{
	struct regmap *rm = lrdata->lora_device;

	/* Drop the frame. */
	mutex_lock(&(lrdata->buf_lock));
	if (lrdata->tx_skb == NULL) {
		sx127X_setState(rm, SX127X_STANDBY_MODE);
		sx127X_setState(rm, SX127X_RXCONTINUOUS_MODE);
	}
	mutex_unlock(&(lrdata->buf_lock));
}

int
sx127X_ieee_start(struct ieee802154_hw *hw)
{
	struct lora_struct *lrdata = hw->priv;
	struct regmap *rm = lrdata->lora_device;

	sx127X_startLoRaMode(rm);

	lrdata->timer_enable = 1;
	add_timer(&(lrdata->timer));

	return 0;
}

void
sx127X_ieee_stop(struct ieee802154_hw *hw)
{
	struct lora_struct *lrdata = hw->priv;
	struct regmap *rm = lrdata->lora_device;

	lrdata->timer_enable = 0;
	sx127X_setState(rm, SX127X_SLEEP_MODE);
}

int
sx127X_ieee_tx(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	struct lora_struct *lrdata = hw->priv;
	struct regmap *rm = lrdata->lora_device;
	int len;
	int ret;

	/* Check TX is not used or used for now. */
	mutex_lock(&(lrdata->buf_lock));
	if (lrdata->tx_skb == NULL) {
		lrdata->tx_skb = skb;
		ret = 0;
	}
	else {
		ret = -EBUSY;
	}
	mutex_unlock(&(lrdata->buf_lock));

	/* Write and fill the FIFO of the chip if TX is not used. */
	if(ret == 0) {
		len = sx127X_sendLoRaData(rm, skb->data, skb->len);
		if (len > 0)
			sx127X_setState(rm, SX127X_TX_MODE);
		else
			ret = len;
	}

	return ret;
}

void
sx127X_ieee_tx_complete(struct lora_struct *lrdata)
{
	struct regmap *rm = lrdata->lora_device;

	mutex_lock(&(lrdata->buf_lock));
	if (lrdata->tx_skb != NULL) {
		ieee802154_xmit_complete(lrdata->hw, lrdata->tx_skb, false);
		lrdata->tx_skb = NULL;
	}
	mutex_unlock(&(lrdata->buf_lock));

	sx127X_setState(rm, SX127X_RXCONTINUOUS_MODE);
}

int
sx127X_ieee_ed(struct ieee802154_hw *hw, u8 *level)
{
	struct lora_struct *lrdata = hw->priv;
	struct regmap *rm = lrdata->lora_device;
	int32_t rssi;
	int32_t range = SX127X_IEEE_ENERGY_RANGE - 10;

	/* ED: IEEE  802.15.4-2011 8.2.5 Recevier ED. */
	rssi = sx127X_getLoRaRSSI(rm);
	if (rssi < (SX127X_IEEE_SENSITIVITY + 10))
		*level = 0;
	else if (rssi >= 0)
		*level = 255;
	else
		*level = ((int32_t)255 * (rssi + range)	/ range) % 255;

	return 0;
}

#ifndef	SX127X_IEEE_CHANNEL_MIN
#define	SX127X_IEEE_CHANNEL_MIN		11
#endif
#ifndef	SX127X_IEEE_CHANNEL_MAX
#define	SX127X_IEEE_CHANNEL_MAX		26
#endif
#ifndef SX127X_IEEE_CENTER_CARRIER_FRQ
#define SX127X_IEEE_CENTER_CARRIER_FRQ	434000000
#endif
#ifndef SX127X_IEEE_BANDWIDTH
#define	SX127X_IEEE_BANDWIDTH		500000
#endif

struct rf_frq {
	uint32_t carrier;
	uint32_t bandwidth;
	uint8_t ch_min;
	uint8_t ch_max;
};

void
sx127X_ieee_get_rf_config(struct ieee802154_hw *hw, struct rf_frq *rf)
{
#ifdef CONFIG_OF
	struct lora_struct *lrdata = hw->priv;
	struct regmap *rm = lrdata->lora_device;
	const void *ptr;

	/* Set the LoRa chip's center carrier frequency. */
	ptr = of_get_property((regmap_get_device(rm))->of_node,
			"center-carrier-frq",
			NULL);
	rf->carrier = (ptr != NULL) ?
			be32_to_cpup(ptr) : SX127X_IEEE_CENTER_CARRIER_FRQ;

	/* Set the LoRa chip's RF bandwidth. */
	ptr = of_get_property((regmap_get_device(rm))->of_node,
			"rf-bandwidth",
			NULL);
	rf->bandwidth = (ptr != NULL) ?
			be32_to_cpup(ptr) : SX127X_IEEE_BANDWIDTH;

	/* Set the LoRa chip's min & max RF channel if OF is defined. */
	ptr = of_get_property((regmap_get_device(rm))->of_node,
			"minimal-RF-channel",
			NULL);
	rf->ch_min = (ptr != NULL) ?
			be32_to_cpup(ptr) : SX127X_IEEE_CHANNEL_MIN;

	ptr = of_get_property((regmap_get_device(rm))->of_node,
			"maximum-RF-channel",
			NULL);
	rf->ch_max = (ptr != NULL) ?
			be32_to_cpup(ptr) : SX127X_IEEE_CHANNEL_MAX;
#else
	rf->carrier = SX127X_IEEE_CENTER_CARRIER_FRQ;
	rf->bandwidth = SX127X_IEEE_BANDWIDTH;
	rf->ch_min = SX127X_IEEE_CHANNEL_MIN;
	rf->ch_max = SX127X_IEEE_CHANNEL_MAX;
#endif
}

int
sx127X_ieee_set_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	struct lora_struct *lrdata = hw->priv;
	struct regmap *rm = lrdata->lora_device;
	struct rf_frq rf;
	uint32_t fr;
	int8_t d;

	sx127X_ieee_get_rf_config(hw, &rf);

	if (channel < rf.ch_min)
		channel = rf.ch_min;
	else if (channel > rf.ch_max)
		channel = rf.ch_max;

	d = channel - (rf.ch_min + rf.ch_max) / 2;
	fr = rf.carrier + d * rf.bandwidth;

	sx127X_setLoRaFreq(rm, fr);

	return 0;
}

int
sx127X_ieee_set_txpower(struct ieee802154_hw *hw, s32 mbm)
{
	struct lora_struct *lrdata = hw->priv;
	struct regmap *rm = lrdata->lora_device;
	int32_t dbm = sx127X_mbm2dbm(mbm);

	sx127X_setLoRaPower(rm, dbm);

	return 0;
}

/* in mbm */
int32_t sx127X_powers[] = {
	-200, -100, 0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100,
	1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300};

/**
 * sx127X_ieee_channel_mask - Get the available channels' mask of LoRa device
 * @hw:		LoRa IEEE 802.15.4 device
 *
 * Return:	The bitwise channel mask in 4 bytes
 */
uint32_t
sx127X_ieee_channel_mask(struct ieee802154_hw *hw)
{
	struct rf_frq rf;
	uint32_t mask;

	sx127X_ieee_get_rf_config(hw, &rf);

	mask = ((uint32_t)(1 << (rf.ch_max + 1)) - (uint32_t)(1 << rf.ch_min));

	return mask;
}

static const struct ieee802154_ops sx127X_ieee_ops = {
	.owner = THIS_MODULE,
	.start = sx127X_ieee_start,
	.stop = sx127X_ieee_stop,
	.xmit_async = sx127X_ieee_tx,
	.ed = sx127X_ieee_ed,
	.set_channel = sx127X_ieee_set_channel,
	.set_txpower = sx127X_ieee_set_txpower,
	//.set_promiscuous_mode = NULL,
};

/**
 * sx127X_ieee_register - Register LoRa device's from IEEE 802.15.4
 * @hw:		LoRa IEEE 802.15.4 device
 *
 * Return:	0 for success, negative number for error
 */
int
sx127X_ieee_register(struct ieee802154_hw *hw)
{
	struct lora_struct *lrdata = hw->priv;
	int ret = -ENOMEM;

	hw->parent = regmap_get_device(lrdata->lora_device);
	hw->extra_tx_headroom = 0;
	ieee802154_random_extended_addr(&(hw->phy->perm_extended_addr));

	/* Define channels could be used. */
	hw->phy->supported.channels[0] = sx127X_ieee_channel_mask(hw);
	hw->phy->current_channel = 11;

	/* Set LoRa sx127X features for IEEE 802.15.4. */
	hw->flags = IEEE802154_HW_TX_OMIT_CKSUM;
	hw->phy->flags = WPAN_PHY_FLAG_TXPOWER;

	/* Define RF power. */
	hw->phy->supported.tx_powers = sx127X_powers;
	hw->phy->supported.tx_powers_size = ARRAY_SIZE(sx127X_powers);
	hw->phy->transmit_power = sx127X_powers[12];

	dev_dbg(regmap_get_device(lrdata->lora_device),
		"register IEEE 802.15.4 LoRa sx127X\n");
	ret = ieee802154_register_hw(hw);
	if (ret)
		dev_err(regmap_get_device(lrdata->lora_device),
			"register as IEEE 802.15.4 device failed\n");

	return ret;
}

/**
 * sx127X_ieee_unregister - Unregister LoRa device's from IEEE 802.15.4
 * @hw:		LoRa IEEE 802.15.4 device
 */
void
sx127X_ieee_unregister(struct ieee802154_hw *hw)
{
	ieee802154_unregister_hw(hw);
}

/*------------------------- SX127X Timer for Polling -------------------------*/

/**
 * sx127X_timer_irqwork - The actual work which checks the IRQ flags of the chip
 * @work:	the work entry listed in the workqueue
 */
void
sx127X_timer_irqwork(struct work_struct *work)
{
	struct lora_struct *lrdata;
	uint8_t flag;

	lrdata = container_of(work, struct lora_struct, irqwork);

	flag = sx127X_getLoRaAllFlag(lrdata->lora_device);
	/* Check RX flags for an incoming new frame. */
	if (((flag & SX127X_FLAG_RXTIMEOUT) == 0)
		&& ((flag & SX127X_FLAG_PAYLOADCRCERROR) == 0)
		&& (flag & SX127X_FLAG_RXDONE)) {
		sx127X_ieee_rx(lrdata);
		sx127X_clearLoRaFlag(lrdata->lora_device, SX127X_FLAG_RXDONE);
	}
	/* Check RX error flags for an incoming frame. */
	else if (flag & (SX127X_FLAG_RXTIMEOUT | SX127X_FLAG_PAYLOADCRCERROR)) {
		sx127X_ieee_rx_error(lrdata);
		sx127X_clearLoRaFlag(lrdata->lora_device,
				SX127X_FLAG_RXTIMEOUT
				| SX127X_FLAG_PAYLOADCRCERROR);
	}
	/* Check TX flags for outgoing frame is done.*/
	if (flag & SX127X_FLAG_TXDONE) {
		sx127X_ieee_tx_complete(lrdata);
		sx127X_clearLoRaFlag(lrdata->lora_device, SX127X_FLAG_TXDONE);
	}

	/* Eanable the LoRa timer again. */
	if (lrdata->timer_enable) {
		lrdata->timer.expires = jiffies + HZ;
		add_timer(&(lrdata->timer));
	}
}

/**
 * sx127X_timer_isr - Callback function for the timer interrupt
 * @arg:	the general argument for this callback function
 */
void
sx127X_timer_isr(unsigned long arg)
{
	struct lora_struct *lrdata = (struct lora_struct *)arg;

	schedule_work(&(lrdata->irqwork));
}

/*---------------------------- LoRa SPI Functions ----------------------------*/

#define __DRIVER_NAME		"sx1278"
#ifndef N_LORASPI_MINORS
#define N_LORASPI_MINORS	8
#endif

static DEFINE_MUTEX(minors_lock);

#ifdef LORA_DEBUG_FS
static DECLARE_BITMAP(minors, N_LORASPI_MINORS);

/**
 * loraspi_read - Read from the LoRa device's communication
 * @lrdata:	LoRa device
 * @arg:	the buffer going to hold the read data in user space
 * @size:	the length of the buffer in bytes
 *
 * Return:	Read how many bytes actually, negative number for error
 */
static ssize_t
loraspi_read(struct lora_struct *lrdata, const char __user *buf, size_t size)
{
	struct regmap *rm;
	ssize_t status;
	int c = 0;
	uint8_t adr;
	uint8_t flag;
	uint8_t len;
	uint8_t st;
	uint32_t timeout;

	rm = lrdata->lora_device;
	dev_dbg(regmap_get_device(rm),
		"Read %zu bytes into user space\n",
		size);

	mutex_lock(&(lrdata->buf_lock));
	/* Get chip's current state. */
	st = sx127X_getState(rm);

	/*  Prepare and set the chip to RX continuous mode, if it is not. */
	if (st != SX127X_RXCONTINUOUS_MODE) {
		/* Set chip to standby state. */
		dev_dbg(regmap_get_device(rm),
			"Going to set standby state\n");
		sx127X_setState(rm, SX127X_STANDBY_MODE);

		/* Set chip FIFO RX base. */
		adr = 0x00;
		dev_dbg(regmap_get_device(rm),
			"Going to set RX base address\n");
		regmap_raw_write(rm, SX127X_REG_FIFO_RX_BASE_ADDR, &adr, 1);

		/* Clear all of the IRQ flags. */
		sx127X_clearLoRaAllFlag(rm);
		/* Set chip to RX continuous state waiting for receiving. */
		sx127X_setState(rm, SX127X_RXCONTINUOUS_MODE);
	}

	/* Wait and check there is any packet received ready. */
	for (timeout = 0; timeout < 250; timeout++) {
		flag = sx127X_getLoRaFlag(rm,
					SX127X_FLAG_RXTIMEOUT |
					SX127X_FLAG_RXDONE |
					SX127X_FLAG_PAYLOADCRCERROR);
		if (flag == 0)
			msleep(20);
		else
			break;
	}

	/* If there is nothing or received timeout. */
	if ((flag == 0) || (flag & SX127X_FLAG_RXTIMEOUT)) {
		c = -ENODATA;
	}
	/* If there is a packet, but the payload is CRC error. */
	if (sx127X_getLoRaFlag(rm, SX127X_FLAG_PAYLOADCRCERROR)) {
		c = -EBADMSG;
	}

	/* There is a ready packet in the chip's FIFO. */
	if (c == 0) {
		memset(lrdata->rx_buf, 0, lrdata->bufmaxlen);
		len = sx127X_getLoRaLastPacketPayloadLen(rm);
		size = (len <= size) ? len : size;
		size = (lrdata->bufmaxlen <= size) ? lrdata->bufmaxlen : size;
		/* Read from chip to LoRa data RX buffer. */
		c = sx127X_readLoRaData(rm, lrdata->rx_buf, size);
		/* Copy from LoRa data RX buffer to user space. */
		if (c > 0)
			status = copy_to_user((void *)buf, lrdata->rx_buf, c);
	}

	/* Clear all of the IRQ flags. */
	sx127X_clearLoRaAllFlag(rm);

	mutex_unlock(&(lrdata->buf_lock));

	return c;
}

/**
 * loraspi_write - Write to the LoRa device's communication
 * @lrdata:	LoRa device
 * @arg:	the buffer holding the data going to be written in user space
 * @size:	the length of the buffer in bytes
 *
 * Return:	Write how many bytes actually, negative number for error
 */
static ssize_t
loraspi_write(struct lora_struct *lrdata, const char __user *buf, size_t size)
{
	struct regmap *rm;
	ssize_t status;
	int c;
	uint8_t adr;
	uint8_t flag;
	uint32_t timeout;

	rm = lrdata->lora_device;
	dev_dbg(regmap_get_device(rm),
		"Write %zu bytes from user space\n",
		size);

	mutex_lock(&(lrdata->buf_lock));
	memset(lrdata->tx_buf, 0, lrdata->bufmaxlen);
	size = (lrdata->bufmaxlen < size) ? lrdata->bufmaxlen : size;
	status = copy_from_user(lrdata->tx_buf, buf, size);

	if (status != 0) {
		mutex_unlock(&(lrdata->buf_lock));
		return 0;
	}

	lrdata->tx_buflen = size - status;

	/* Set chip to standby state. */
	dev_dbg(regmap_get_device(rm), "Going to set standby state\n");
	sx127X_setState(rm, SX127X_STANDBY_MODE);

	/* Set chip FIFO TX base. */
	adr = 0x80;
	dev_dbg(regmap_get_device(rm), "Going to set TX base address\n");
	regmap_raw_write(rm, SX127X_REG_FIFO_TX_BASE_ADDR, &adr, 1);

	/* Write to SPI chip synchronously to fill the FIFO of the chip. */
	c = sx127X_sendLoRaData(rm, lrdata->tx_buf, lrdata->tx_buflen);

	/* Clear LoRa IRQ TX flag. */
	sx127X_clearLoRaFlag(rm, SX127X_FLAG_TXDONE);

	if (c > 0) {
		/* Set chip to TX state to send the data in FIFO to RF. */
		dev_dbg(regmap_get_device(rm), "Set TX state\n");
		sx127X_setState(rm, SX127X_TX_MODE);

		timeout = (c + sx127X_getLoRaPreambleLen(rm) + 1) + 2;
		dev_dbg(regmap_get_device(rm),
			"The time out is %u ms",
			timeout * 20);

		/* Wait until TX is finished by checking the TX flag. */
		for (flag = 0; timeout > 0; timeout--) {
			flag = sx127X_getLoRaFlag(rm, SX127X_FLAG_TXDONE);
			if (flag != 0) {
				dev_dbg(regmap_get_device(rm),
					"Wait TX is finished\n");
				break;
			}

			if (timeout == 1) {
				c = 0;
				dev_dbg(regmap_get_device(rm),
					"Wait TX is time out\n");
			}
			else {
				msleep(20);
			}
		}
	}

	/* Set chip to RX continuous state. */
	dev_dbg(regmap_get_device(rm), "Set back to RX continuous state\n");
	sx127X_setState(rm, SX127X_STANDBY_MODE);
	sx127X_setState(rm, SX127X_RXCONTINUOUS_MODE);

	lrdata->tx_buflen = 0;

	mutex_unlock(&(lrdata->buf_lock));

	return c;
}

/**
 * loraspi_setstate - Set the state of the LoRa device
 * @lrdata:	LoRa device
 * @arg:	the buffer holding the state value in user space
 *
 * Return:	0 / other values for success / error
 */
static long
loraspi_setstate(struct lora_struct *lrdata, void __user *arg)
{
	struct regmap *rm;
	int status;
	uint32_t st32;
	uint8_t st;

	rm = lrdata->lora_device;
	status = copy_from_user(&st32, arg, sizeof(uint32_t));
	switch (st32) {
	case LORA_STATE_SLEEP:
		st = SX127X_SLEEP_MODE;		break;
	case LORA_STATE_STANDBY:
		st = SX127X_STANDBY_MODE;	break;
	case LORA_STATE_TX:
		st = SX127X_TX_MODE;		break;
	case LORA_STATE_RX:
		st = SX127X_RXCONTINUOUS_MODE;	break;
	case LORA_STATE_CAD:
		st = SX127X_CAD_MODE;		break;
	default:
		st = SX127X_STANDBY_MODE;
	}

	mutex_lock(&(lrdata->buf_lock));
	sx127X_setState(rm, st);
	mutex_unlock(&(lrdata->buf_lock));

	return 0;
}

/**
 * loraspi_getstate - Get the state of the LoRa device
 * @lrdata:	LoRa device
 * @arg:	the buffer going to hold the state value in user space
 *
 * Return:	0 / other values for success / error
 */
static long
loraspi_getstate(struct lora_struct *lrdata, void __user *arg)
{
	struct regmap *rm;
	int status;
	uint32_t st32;
	uint8_t st;

	rm = lrdata->lora_device;

	mutex_lock(&(lrdata->buf_lock));
	st = sx127X_getState(rm);
	mutex_unlock(&(lrdata->buf_lock));

	st32 = st;
	switch (st) {
	case SX127X_SLEEP_MODE:
		st32 = LORA_STATE_SLEEP;	break;
	case SX127X_STANDBY_MODE:
		st32 = LORA_STATE_STANDBY;	break;
	case SX127X_FSTX_MODE:
	case SX127X_TX_MODE:
		st32 = LORA_STATE_TX;		break;
	case SX127X_FSRX_MODE:
	case SX127X_RXSINGLE_MODE:
	case SX127X_RXCONTINUOUS_MODE:
		st32 = LORA_STATE_RX;		break;
	case SX127X_CAD_MODE:
		st32 = LORA_STATE_CAD;		break;
	default:
		st32 = LORA_STATE_SLEEP;
	}
	status = copy_to_user(arg, &st32, sizeof(uint32_t));

	return 0;
}

/**
 * loraspi_setfreq - Set the carrier frequency
 * @lrdata:	LoRa device
 * @arg:	the buffer holding the carrier frequency in user space
 *
 * Return:	0 / other values for success / error
 */
static long
loraspi_setfreq(struct lora_struct *lrdata, void __user *arg)
{
	struct regmap *rm;
	int status;
	uint32_t freq;

	rm = lrdata->lora_device;
	status = copy_from_user(&freq, arg, sizeof(uint32_t));
	dev_dbg(regmap_get_device(rm),
		"Set frequency %u Hz from user space\n",
		freq);

	mutex_lock(&(lrdata->buf_lock));
	sx127X_setLoRaFreq(rm, freq);
	mutex_unlock(&(lrdata->buf_lock));

	return 0;
}

/**
 * loraspi_getfreq - Get the carrier frequency
 * @lrdata:	LoRa device
 * @arg:	the buffer going to hold the carrier frequency in user space
 *
 * Return:	0 / other values for success / error
 */
static long
loraspi_getfreq(struct lora_struct *lrdata, void __user *arg)
{
	struct regmap *rm;
	int status;
	uint32_t freq;

	rm = lrdata->lora_device;
	dev_dbg(regmap_get_device(rm), "Get frequency to user space\n");

	mutex_lock(&(lrdata->buf_lock));
	freq = sx127X_getLoRaFreq(rm);
	mutex_unlock(&(lrdata->buf_lock));
	dev_dbg(regmap_get_device(rm), "The carrier freq is %u Hz\n", freq);

	status = copy_to_user(arg, &freq, sizeof(uint32_t));

	return 0;
}

/**
 * loraspi_setpower - Set the PA power
 * @lrdata:	LoRa device
 * @arg:	the buffer holding the PA output value in user space
 *
 * Return:	0 / other values for success / error
 */
static long
loraspi_setpower(struct lora_struct *lrdata, void __user *arg)
{
	struct regmap *rm;
	int status;
	int32_t dbm;

	rm = lrdata->lora_device;
	status = copy_from_user(&dbm, arg, sizeof(int32_t));

#define LORA_MAX_POWER	(17)
#define LORA_MIN_POWER	(-2)
	if (dbm > LORA_MAX_POWER)
		dbm = LORA_MAX_POWER;
	else if (dbm < LORA_MIN_POWER)
		dbm = LORA_MIN_POWER;

	mutex_lock(&(lrdata->buf_lock));
	sx127X_setLoRaPower(rm, dbm);
	mutex_unlock(&(lrdata->buf_lock));

	return 0;
}

/**
 * loraspi_getpower -  Get the PA power
 * @lrdata:	LoRa device
 * @arg:	the buffer going to hold the PA output value in user space
 *
 * Return:	0 / other values for success / error
 */
static long
loraspi_getpower(struct lora_struct *lrdata, void __user *arg)
{
	struct regmap *rm;
	int status;
	int32_t dbm;

	rm = lrdata->lora_device;

	mutex_lock(&(lrdata->buf_lock));
	dbm = sx127X_getLoRaPower(rm);
	mutex_unlock(&(lrdata->buf_lock));

	status = copy_to_user(arg, &dbm, sizeof(int32_t));

	return 0;
}

/**
 * loraspi_setLNA - Set the LNA gain
 * @lrdata:	LoRa device
 * @arg:	the buffer holding the LNA gain value in user space
 *
 * Return:	0 / other values for success / error
 */
static long
loraspi_setLNA(struct lora_struct *lrdata, void __user *arg)
{
	struct regmap *rm;
	int status;
	int32_t db;

	rm = lrdata->lora_device;
	status = copy_from_user(&db, arg, sizeof(int32_t));

#define LORA_MAX_LNA	(0)
#define LORA_MIN_LNA	(-48)
	if (db > LORA_MAX_LNA)
		db = LORA_MAX_LNA;
	else if (db < LORA_MIN_LNA)
		db = LORA_MIN_LNA;

	mutex_lock(&(lrdata->buf_lock));
	sx127X_setLoRaLNA(rm, db);
	mutex_unlock(&(lrdata->buf_lock));

	return 0;
}

/**
 * loraspi_getLNA -  Get the LNA gain
 * @lrdata:	LoRa device
 * @arg:	the buffer going to hold the LNA gain value in user space
 *
 * Return:	0 / other values for success / error
 */
static long
loraspi_getLNA(struct lora_struct *lrdata, void __user *arg)
{
	struct regmap *rm;
	int status;
	int32_t db;

	rm = lrdata->lora_device;

	mutex_lock(&(lrdata->buf_lock));
	db = sx127X_getLoRaLNA(rm);
	mutex_unlock(&(lrdata->buf_lock));

	status = copy_to_user(arg, &db, sizeof(int32_t));

	return 0;
}

/**
 * loraspi_setLNAAGC - Set the LNA be auto gain control
 * @lrdata:	LoRa device
 * @arg:	the buffer holding the auto gain control or manual in user space
 *
 * Return:	0 / other values for success / error
 */
static long
loraspi_setLNAAGC(struct lora_struct *lrdata, void __user *arg)
{
	struct regmap *rm;
	int status;
	uint32_t agc;

	rm = lrdata->lora_device;
	status = copy_from_user(&agc, arg, sizeof(uint32_t));

	agc = (agc == 1) ? 1 : 0;
	mutex_lock(&(lrdata->buf_lock));
	sx127X_setLoRaLNAAGC(rm, agc);
	mutex_unlock(&(lrdata->buf_lock));

	return 0;
}

/**
 * loraspi_setsprfactor - Set the RF spreading factor
 * @lrdata:	LoRa device
 * @arg:	the buffer holding the spreading factor in user space
 *
 * Return:	0 / other values for success / error
 */
static long
loraspi_setsprfactor(struct lora_struct *lrdata, void __user *arg)
{
	struct regmap *rm;
	int status;
	uint32_t sprf;

	rm = lrdata->lora_device;
	status = copy_from_user(&sprf, arg, sizeof(uint32_t));

	mutex_lock(&(lrdata->buf_lock));
	sx127X_setLoRaSPRFactor(rm, sprf);
	mutex_unlock(&(lrdata->buf_lock));

	return 0;
}

/**
 * loraspi_getsprfactor - Get the RF spreading factor
 * @lrdata:	LoRa device
 * @arg:	the buffer going to hold the spreading factor in user space
 *
 * Return:	0 / other values for success / error
 */
static long
loraspi_getsprfactor(struct lora_struct *lrdata, void __user *arg)
{
	struct regmap *rm;
	int status;
	uint32_t sprf;

	rm = lrdata->lora_device;

	mutex_lock(&(lrdata->buf_lock));
	sprf = sx127X_getLoRaSPRFactor(rm);
	mutex_unlock(&(lrdata->buf_lock));

	status = copy_to_user(arg, &sprf, sizeof(uint32_t));

	return 0;
}

/**
 * loraspi_setbandwidth - Set the RF bandwith
 * @lrdata:	LoRa device
 * @arg:	the buffer holding the RF bandwith value in user space
 *
 * Return:	0 / other values for success / error
 */
static long
loraspi_setbandwidth(struct lora_struct *lrdata, void __user *arg)
{
	struct regmap *rm;
	int status;
	uint32_t bw;

	rm = lrdata->lora_device;
	status = copy_from_user(&bw, arg, sizeof(uint32_t));

	mutex_lock(&(lrdata->buf_lock));
	sx127X_setLoRaBW(rm, bw);
	mutex_unlock(&(lrdata->buf_lock));

	return 0;
}

/**
 * loraspi_getbandwidth - Get the RF bandwith
 * @lrdata:	LoRa device
 * @arg:	the buffer going to hold the RF bandwith value in user space
 *
 * Return:	0 / other values for success / error
 */
static long
loraspi_getbandwidth(struct lora_struct *lrdata, void __user *arg)
{
	struct regmap *rm;
	int status;
	uint32_t bw;

	rm = lrdata->lora_device;

	mutex_lock(&(lrdata->buf_lock));
	bw = sx127X_getLoRaBW(rm);
	mutex_unlock(&(lrdata->buf_lock));

	status = copy_to_user(arg, &bw, sizeof(uint32_t));

	return 0;
}

/**
 * loraspi_getrssi - Get current RSSI
 * @lrdata:	LoRa device
 * @arg:	the buffer going to hold the RSSI value in user space
 *
 * Return:	0 / other values for success / error
 */
static long
loraspi_getrssi(struct lora_struct *lrdata, void __user *arg)
{
	struct regmap *rm;
	int status;
	int32_t rssi;

	rm = lrdata->lora_device;

	mutex_lock(&(lrdata->buf_lock));
	rssi = sx127X_getLoRaRSSI(rm);
	mutex_unlock(&(lrdata->buf_lock));

	status = copy_to_user(arg, &rssi, sizeof(int32_t));

	return 0;
}

/**
 * loraspi_getsnr - Get last packet's SNR
 * @lrdata:	LoRa device
 * @arg:	the buffer going to hold the SNR value in user space
 *
 * Return:	0 / other values for success / error
 */
static long
loraspi_getsnr(struct lora_struct *lrdata, void __user *arg)
{
	struct regmap *rm;
	int status;
	int32_t snr;

	rm = lrdata->lora_device;

	mutex_lock(&(lrdata->buf_lock));
	snr = sx127X_getLoRaLastPacketSNR(rm);
	mutex_unlock(&(lrdata->buf_lock));

	status = copy_to_user(arg, &snr, sizeof(int32_t));

	return 0;
}

/**
 * loraspi_ready2write - Is ready to be written
 * @lrdata:	LoRa device
 *
 * Return:	1 / 0 for ready / not ready
 */
static long
loraspi_ready2write(struct lora_struct *lrdata)
{
	long ret;

	/* Mutex is not lock, than it is not writing. */
	ret = mutex_is_locked(&(lrdata->buf_lock)) ? 0 : 1;

	return ret;
}

/**
 * loraspi_ready2read - Is ready to be read
 * @lrdata:	LoRa device
 *
 * Return:	1 / 0 for ready / not ready
 */
static long
loraspi_ready2read(struct lora_struct *lrdata)
{
	struct regmap *rm;
	long ret;

	rm = lrdata->lora_device;

	ret = 0;
	/* Mutex is not lock, than it is not in reading file operation. */
	if (!mutex_is_locked(&(lrdata->buf_lock))) {
		/* Check the chip have recieved full data. */
		mutex_lock(&(lrdata->buf_lock));
		ret = sx127X_getLoRaFlag(rm, SX127X_FLAG_RXDONE) != 0;
		mutex_unlock(&(lrdata->buf_lock));
	}

	return ret;
}

struct lora_driver lr_driver = {
	.name = __DRIVER_NAME,
	.num = N_LORASPI_MINORS,
	.owner = THIS_MODULE,
};

struct lora_operations lrops = {
	.read = loraspi_read,
	.write = loraspi_write,
	.setState = loraspi_setstate,
	.getState = loraspi_getstate,
	.setFreq = loraspi_setfreq,
	.getFreq = loraspi_getfreq,
	.setPower = loraspi_setpower,
	.getPower = loraspi_getpower,
	.setLNA = loraspi_setLNA,
	.getLNA = loraspi_getLNA,
	.setLNAAGC = loraspi_setLNAAGC,
	.setSPRFactor = loraspi_setsprfactor,
	.getSPRFactor = loraspi_getsprfactor,
	.setBW = loraspi_setbandwidth,
	.getBW = loraspi_getbandwidth,
	.getRSSI = loraspi_getrssi,
	.getSNR = loraspi_getsnr,
	.ready2write = loraspi_ready2write,
	.ready2read = loraspi_ready2read,
};
#endif

/* The compatible SoC array. */
#ifdef CONFIG_OF
static const struct of_device_id lora_dt_ids[] = {
	{ .compatible = "semtech,sx1276" },
	{ .compatible = "semtech,sx1277" },
	{ .compatible = "semtech,sx1278" },
	{ .compatible = "semtech,sx1279" },
	{ .compatible = "sx1278" },
	{},
};
MODULE_DEVICE_TABLE(of, lora_dt_ids);
#endif

#ifdef CONFIG_ACPI

/* The compatible ACPI device array. */
#define LORA_ACPI_DUMMY	1
static const struct acpi_device_id lora_acpi_ids[] = {
	{ .id = "sx1278" },
	{},
};
MODULE_DEVICE_TABLE(acpi, lora_acpi_ids);

/* The callback function of ACPI probes LoRa SPI. */
static void loraspi_probe_acpi(struct spi_device *spi) {
	const struct acpi_device_id *id;

	if (!has_acpi_companion(&(spi->dev)))
		return;

	id = acpi_match_device(lora_acpi_ids, &(spi->dev));
	if (WARN_ON(!id))
		return;

	if (id->driver_data == LORA_ACPI_DUMMY)
		dev_warn(&(spi->dev),
			"Do not use this driver in produciton systems.\n");
}
#else
static void loraspi_probe_acpi(struct spi_device *spi) {};
#endif

/* The compatible SPI device id array. */
static const struct spi_device_id spi_ids[] = {
	{ .name = "sx1278" },
	{},
};
MODULE_DEVICE_TABLE(spi, spi_ids);

bool sx127X_reg_volatile(struct device *dev, unsigned int reg)
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
	.volatile_reg = sx127X_reg_volatile,
};

/* The SPI probe callback function. */
static int loraspi_probe(struct spi_device *spi)
{
	struct lora_struct *lrdata;
	struct ieee802154_hw *hw;
#ifdef LORA_DEBUG_FS
	struct device *dev;
	unsigned long minor;
#endif
	int v;
	int status = 0;

	dev_dbg(&(spi->dev), "probe a LoRa SPI device\n");

#ifdef CONFIG_OF
	if (spi->dev.of_node && !of_match_device(lora_dt_ids, &(spi->dev))) {
		dev_err(&(spi->dev), "buggy DT: LoRa listed directly in DT\n");
		WARN_ON(spi->dev.of_node &&
			!of_match_device(lora_dt_ids, &(spi->dev)));
	}
#endif

	loraspi_probe_acpi(spi);

	/* Allocate IEEE 802.15.4 LoRa device's data. */
	hw = ieee802154_alloc_hw(sizeof(*lrdata), &sx127X_ieee_ops);
	if (!hw) {
		dev_err(&(spi->dev), "not enough memory\n");
		return -ENOMEM;
	}
	lrdata = hw->priv;
	lrdata->hw = hw;

	/* Initial the LoRa device's data. */
#ifdef LORA_DEBUG_FS
	lrdata->ops = &lrops;
#endif
	lrdata->lora_device = devm_regmap_init_spi(spi, &sx1278_regmap_config);
	if (IS_ERR(lrdata->lora_device)) {
		status = PTR_ERR(lrdata->lora_device);
		dev_err(&(spi->dev), "regmap_init() failed: %d\n", status);
		ieee802154_free_hw(lrdata->hw);
		return status;
	}

	/* Set the SPI device's driver data for later usage. */
	spi_set_drvdata(spi, lrdata);

	/* Initial the SX127X chip. */
	v = init_sx127X(lrdata->lora_device);
	if(v < 0) {
		status = v;
		dev_err(&(spi->dev), "no LoRa SPI device, error: %d\n", status);
		ieee802154_free_hw(lrdata->hw);
		return status;
	}
	dev_info(&(spi->dev), "probe a LoRa SPI device with chip ver. %d.%d\n",
			(v >> 4) & 0xF,
			v & 0xF);

	mutex_init(&(lrdata->buf_lock));
	lrdata->tx_skb = NULL;
#ifdef LORA_DEBUG_FS
	mutex_lock(&minors_lock);
	minor = find_first_zero_bit(minors, N_LORASPI_MINORS);
	if (minor < N_LORASPI_MINORS) {
		set_bit(minor, minors);
		lrdata->devt = MKDEV(lr_driver.major, minor);
		dev = device_create(lr_driver.lora_class,
				&(spi->dev),
				lrdata->devt,
				lrdata,
				"loraSPI%d.%d",
				spi->master->bus_num, spi->chip_select);

		lora_device_add(lrdata);
		status = PTR_ERR_OR_ZERO(dev);
	}
	else {
		/* No more lora device available. */
		ieee802154_free_hw(lrdata->hw);
		status = -ENODEV;
	}
	mutex_unlock(&minors_lock);
#endif

	status = sx127X_ieee_register(lrdata->hw);
	if (status) {
		sx127X_ieee_unregister(lrdata->hw);
		return status;
	}

	INIT_WORK(&(lrdata->irqwork), sx127X_timer_irqwork);

	init_timer(&(lrdata->timer));
	lrdata->timer.expires = jiffies + HZ;
	lrdata->timer.function = sx127X_timer_isr;
	lrdata->timer.data = (unsigned long)lrdata;

	return status;
}

/* The SPI remove callback function. */
static int loraspi_remove(struct spi_device *spi)
{
	struct lora_struct *lrdata;

	dev_info(&(spi->dev), "remove a LoRa SPI device");

	lrdata = spi_get_drvdata(spi);

	/* Remove the timer. */
	del_timer(&(lrdata->timer));
	/* Flush all works in work queue. */
	flush_work(&(lrdata->irqwork));
	/* Clear the lora device's data. */
	lrdata->lora_device = NULL;
#ifdef LORA_DEBUG_FS
	/* No more operations to the lora device from user space. */
	lora_device_remove(lrdata);
	mutex_lock(&minors_lock);
	device_destroy(lr_driver.lora_class, lrdata->devt);
	clear_bit(MINOR(lrdata->devt), minors);
	mutex_unlock(&minors_lock);
#endif
	/* Unregister IEEE LoRa device. */
	sx127X_ieee_unregister(lrdata->hw);
	/* Set the SX127X chip to sleep. */
	sx127X_setState(dev_get_regmap(&(spi->dev), NULL), SX127X_SLEEP_MODE);

	/* Free the memory of the IEEE 802.15.4 LoRa device.  */
	ieee802154_free_hw(lrdata->hw);

	return 0;
}

/* The SPI driver which acts as a protocol driver in this kernel module. */
static struct spi_driver lora_spi_driver = {
	.driver = {
		.name = __DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = lora_dt_ids,
#endif
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(lora_acpi_ids),
#endif
	},
	.probe = loraspi_probe,
	.remove = loraspi_remove,
	.id_table = spi_ids,
};


/* LoRa-SPI kernel module's initial function. */
static int loraspi_sx1278_init(void)
{
	int status;

	pr_debug("sx1278: init SX1278 compatible kernel module\n");

#ifdef LORA_DEBUG_FS
	/* Register a kind of LoRa driver. */
	lora_register_driver(&lr_driver);
#endif

	/* Register LoRa SPI driver as an SPI driver. */
	status = spi_register_driver(&lora_spi_driver);

	return status;
}

/* LoRa-SPI kernel module's exit function. */
static void loraspi_sx1278_exit(void)
{
	pr_debug("sx1278: exit\n");

	/* Unregister the LoRa SPI driver. */
	spi_unregister_driver(&lora_spi_driver);
#ifdef LORA_DEBUG_FS
	/* Unregister the lora driver. */
	lora_unregister_driver(&lr_driver);
#endif
}

module_init(loraspi_sx1278_init);
module_exit(loraspi_sx1278_exit);

MODULE_AUTHOR("Jian-Hong Pan, <starnight@g.ncu.edu.tw>");
MODULE_DESCRIPTION("LoRa device driver with SPI interface");
MODULE_LICENSE("Dual BSD/GPL");
