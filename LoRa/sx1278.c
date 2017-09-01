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


#define SX127X_FIFO_RX_BASE_ADDRESS		0x00
#define SX127X_FIFO_TX_BASE_ADDRESS		0x80

struct sx1278_phy {
	struct ieee802154_hw *hw;
	struct spi_device *spi;
	struct regmap *rm;

	u8 page;
	u8 channel;

	bool suspended;
	u8 opmode;
	struct timer_list timer;
	struct work_struct irqwork;

	bool one_to_be_sent;
	struct spi_message tx_base_msg;
	u8 tx_base_adr[2];
	struct spi_transfer tx_base_t;

	struct spi_message tx_buf_msg;
	u8 tx_buf_adr;
	struct sk_buff *tx_buf;
	struct spi_transfer tx_buf_t[2];

	struct spi_message tx_len_msg;
	u8 tx_len[2];
	struct spi_transfer tx_len_t;

	struct spi_message tx_state_msg;
	u8 tx_state[2];
	struct spi_transfer tx_state_t;

	uint8_t tx_delay;

	spinlock_t buf_lock;
	bool is_busy;
};

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
	start_adr = SX127X_FIFO_RX_BASE_ADDRESS;
	/* Set chip FIFO pointer to FIFO last packet address. */
	regmap_raw_write(rm, SX127X_REG_FIFO_ADDR_PTR, &start_adr, 1);

	/* Read LoRa packet payload. */
	ret = regmap_raw_read(rm, SX127X_REG_FIFO, buf, len);

	regmap_raw_write(rm, SX127X_REG_FIFO_ADDR_PTR, &start_adr, 1);

	dev_dbg(regmap_get_device(rm),
		"read %zu bytes from 0x%u with ret=%d\n", len, start_adr, ret);

	return (ret >= 0) ? len : ret;
}

void
sx127X_setLoRaTXState(void *context)
{
	struct sx1278_phy *phy = context;

	spi_message_init(&(phy->tx_state_msg));

	phy->tx_state[0] = SX127X_REG_OP_MODE | 0x80;
	phy->tx_state[1] = (phy->opmode & 0xF8) | SX127X_TX_MODE;
	phy->tx_state_t.tx_buf = phy->tx_state;
	phy->tx_state_t.len = 2;
	spi_message_add_tail(&(phy->tx_state_t), &(phy->tx_state_msg));

	spi_async(phy->spi, &(phy->tx_state_msg));
	dev_dbg(&(phy->spi->dev), "set LoRa TX mode\n");
}

void
sx127X_setLoRaTXFIFOLen(void *context)
{
	struct sx1278_phy *phy = context;

	spi_message_init(&(phy->tx_len_msg));

	phy->tx_len[0] = SX127X_REG_PAYLOAD_LENGTH | 0x80;
	phy->tx_len_t.tx_buf = phy->tx_len;
	phy->tx_len_t.len = 2;
	spi_message_add_tail(&(phy->tx_len_t), &(phy->tx_len_msg));

	phy->tx_len_msg.complete = sx127X_setLoRaTXState;
	phy->tx_len_msg.context = phy;

	spi_async(phy->spi, &(phy->tx_len_msg));
}

/**
 * sx127X_sendLoRaData - Send data out through LoRa device (TX)
 * @rm:		the device as a regmap to communicate with
 * @buf:	buffer going to be send
 * @len:	the length of the buffer in bytes
 *
 * Return:	the actual length written into the LoRa device in bytes
 */
void
sx127X_sendLoRaData(void *context)
{
	struct sx1278_phy *phy = context;

	spi_message_init(&(phy->tx_buf_msg));

	phy->tx_buf_adr = SX127X_REG_FIFO | 0x80;
	phy->tx_buf_t[0].tx_buf = &(phy->tx_buf_adr);
	phy->tx_buf_t[0].len = 1;
	spi_message_add_tail(&(phy->tx_buf_t[0]), &(phy->tx_buf_msg));

	if (phy->tx_buf->len <= IEEE802154_MTU)
		phy->tx_len[1] = phy->tx_buf->len;
	else
		phy->tx_len[1] = IEEE802154_MTU;
	phy->tx_buf_t[1].tx_buf = phy->tx_buf->data;
	phy->tx_buf_t[1].len = phy->tx_len[1];
	spi_message_add_tail(&(phy->tx_buf_t[1]), &(phy->tx_buf_msg));

	phy->tx_buf_msg.complete = sx127X_setLoRaTXFIFOLen;
	phy->tx_buf_msg.context = phy;

	spi_async(phy->spi, &(phy->tx_buf_msg));
}

int
sx127X_setLoRaTXFIFOPTR(void *context)
{
	struct sx1278_phy *phy = context;

	spi_message_init(&(phy->tx_base_msg));

	phy->tx_base_adr[0] = SX127X_REG_FIFO_ADDR_PTR | 0x80;
	phy->tx_base_adr[1] = SX127X_FIFO_TX_BASE_ADDRESS;
	phy->tx_base_t.tx_buf = phy->tx_base_adr;
	phy->tx_base_t.len = 2;

	spi_message_add_tail(&(phy->tx_base_t), &(phy->tx_base_msg));
	phy->tx_base_msg.complete = sx127X_sendLoRaData;
	phy->tx_base_msg.context = phy;

	return spi_async(phy->spi, &(phy->tx_base_msg));
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
	base_adr = SX127X_FIFO_RX_BASE_ADDRESS;
	dev_dbg(regmap_get_device(rm), "going to set RX base address\n");
	regmap_raw_write(rm, SX127X_REG_FIFO_RX_BASE_ADDR, &base_adr, 1);
	/* Set chip FIFO TX base. */
	base_adr = SX127X_FIFO_TX_BASE_ADDRESS;
	dev_dbg(regmap_get_device(rm), "going to set TX base address\n");
	regmap_raw_write(rm, SX127X_REG_FIFO_TX_BASE_ADDR, &base_adr, 1);

	sx127X_setLoRaRXTimeout(rm, 1000);
	sx127X_setLoRaSPRFactor(rm, 512);

	/* Clear all of the IRQ flags. */
	sx127X_clearLoRaAllFlag(rm);
	/* Set chip to RX state waiting for receiving. */
	sx127X_setState(rm, SX127X_RXSINGLE_MODE);
}

/**
 * init_sx127X - Initial the SX127X device
 * @rm:		the device as a regmap to communicate with
 *
 * Return:	0 / negtive values for success / failed
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
		return 0;
	}
	else {
		return -ENODEV;
	}

}

/*---------------------- SX1278 IEEE 802.15.4 Functions ----------------------*/

static int sx1278_ieee_ed(struct ieee802154_hw *hw, u8 *level)
{
	BUG_ON(!level);
	*level = 0xbe;

	return 0;
}

static int sx1278_ieee_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	struct sx1278_phy *phy = hw->priv;

	phy->page = page;
	phy->channel = channel;

	return 0;
}

static int sx1278_ieee_rx(struct ieee802154_hw *hw)
{
	struct sx1278_phy *phy = hw->priv;
	struct sk_buff *skb;
	uint8_t len;
	int err;

	dev_dbg(hw->parent, "%s\n", __func__);
	skb = dev_alloc_skb(IEEE802154_MTU);
	if (!skb) {
		dev_err(regmap_get_device(phy->rm),
			"not enough memory for new incoming frame\n");
		err = -ENOMEM;
		goto sx1278_ieee_rx_err;
	}

	len = sx127X_getLoRaLastPacketPayloadLen(phy->rm);
	sx127X_readLoRaData(phy->rm, skb_put(skb, len), len);
	ieee802154_rx_irqsafe(hw, skb, 0xcc);

	spin_lock(&phy->buf_lock);
	phy->is_busy = false;
	spin_unlock(&phy->buf_lock);
#ifdef DEBUG
	print_hex_dump(KERN_DEBUG, "sx1278 rx: ", DUMP_PREFIX_OFFSET, 16, 1,
						skb->data, skb->len, 0);
#endif
	return 0;

sx1278_ieee_rx_err:
	return err;
}

static int sx1278_ieee_tx_complete(struct ieee802154_hw *hw)
{
	struct sx1278_phy *phy = hw->priv;
	struct sk_buff *skb = phy->tx_buf;

	ieee802154_xmit_complete(hw, skb, false);

#ifdef DEBUG
	print_hex_dump(KERN_DEBUG, "sx1278 tx: ",
			DUMP_PREFIX_OFFSET, 16, 1,
			phy->tx_buf->data, phy->tx_buf->len, 0);
#endif

	spin_lock(&(phy->buf_lock));
	phy->is_busy = false;
	phy->tx_buf = NULL;
	spin_unlock(&(phy->buf_lock));

	return 0;
}

static int sx1278_ieee_xmit(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	struct sx1278_phy *phy = hw->priv;
	int ret;

	WARN_ON(phy->suspended);

	spin_lock(&(phy->buf_lock));
	if (phy->tx_buf) {
		ret = -EBUSY;
	}
	else {
		phy->tx_buf = skb;
		phy->one_to_be_sent = true;
		ret = 0;
	}
	spin_unlock(&(phy->buf_lock));

	return ret;
}

static int sx1278_ieee_start(struct ieee802154_hw *hw)
{
	struct sx1278_phy *phy = hw->priv;

	phy->suspended = false;
	sx127X_startLoRaMode(phy->rm);
	phy->opmode = sx127X_getMode(phy->rm);
	add_timer(&(phy->timer));

	return 0;
}

static void sx1278_ieee_stop(struct ieee802154_hw *hw)
{
	struct sx1278_phy *phy = hw->priv;

	phy->suspended = true;
	del_timer(&(phy->timer));
	sx127X_setState(phy->rm, SX127X_SLEEP_MODE);
}

static int
sx1278_set_promiscuous_mode(struct ieee802154_hw *hw, const bool on)
{
	return 0;
}

static void sx1278_timer_irqwork(struct work_struct *work)
{
	struct sx1278_phy *phy;
	u8 flags;
	u8 state;
	bool do_next_rx = false;

	phy = container_of(work, struct sx1278_phy, irqwork);
	flags = sx127X_getLoRaAllFlag(phy->rm);
	state = sx127X_getState(phy->rm);

	if (flags & (SX127X_FLAG_RXTIMEOUT | SX127X_FLAG_PAYLOADCRCERROR)) {
		sx127X_clearLoRaFlag(phy->rm, SX127X_FLAG_RXTIMEOUT
						| SX127X_FLAG_PAYLOADCRCERROR
						| SX127X_FLAG_RXDONE);
		spin_lock(&(phy->buf_lock));
		phy->is_busy = false;
		spin_unlock(&(phy->buf_lock));
		do_next_rx = true;
	}
	else if (flags & SX127X_FLAG_RXDONE) {
		switch(sx1278_ieee_rx(phy->hw)) {
		case -EBUSY:
			break;
		case 0:
		default:
			sx127X_clearLoRaFlag(phy->rm, SX127X_FLAG_RXDONE);
			do_next_rx = true;
		}
	}
	if (flags & SX127X_FLAG_TXDONE) {
		sx1278_ieee_tx_complete(phy->hw);
		sx127X_clearLoRaFlag(phy->rm, SX127X_FLAG_TXDONE);
		phy->tx_delay = 2;
		do_next_rx = true;
	}

	if (phy->one_to_be_sent && (state == SX127X_STANDBY_MODE) && (phy->tx_delay == 0)) {
		spin_lock(&(phy->buf_lock));
		if (!phy->is_busy) {
			phy->is_busy = true;
			phy->one_to_be_sent = false;
		}
		spin_unlock(&(phy->buf_lock));

		if (!phy->one_to_be_sent) {
			sx127X_setLoRaTXFIFOPTR(phy);
			do_next_rx = false;
		}
	}

	if (do_next_rx) {
		spin_lock(&(phy->buf_lock));
		if (!phy->is_busy) {
			phy->is_busy = true;
			do_next_rx = true;
		}
		else {
			do_next_rx = false;
		}
		spin_unlock(&(phy->buf_lock));

		if (do_next_rx) {
			sx127X_setState(phy->rm, SX127X_RXSINGLE_MODE);
		}
	}

	if (phy->tx_delay > 0) {
		phy->tx_delay -= 1;
	}

	if (!phy->suspended) {
		phy->timer.expires = jiffies + HZ * 20 / 1000;
		add_timer(&(phy->timer));
	}

	return;
}

static void sx1278_timer_isr(unsigned long arg)
{
	struct sx1278_phy *phy = (struct sx1278_phy *)arg;

	schedule_work(&(phy->irqwork));
}

static const struct ieee802154_ops sx1278_ops = {
	.owner = THIS_MODULE,
	.xmit_async = sx1278_ieee_xmit,
	.ed = sx1278_ieee_ed,
	.set_channel = sx1278_ieee_channel,
	.start = sx1278_ieee_start,
	.stop = sx1278_ieee_stop,
	.set_promiscuous_mode = sx1278_set_promiscuous_mode,
};

struct rf_frq {
	uint32_t carrier;
	uint32_t bandwidth;
	uint8_t ch_min;
	uint8_t ch_max;
};

uint32_t
sx1278_ieee_channel_mask(struct ieee802154_hw *hw)
{
	struct rf_frq rf;
	uint32_t mask;

	//sx127X_ieee_get_rf_config(hw, &rf);
	rf.ch_max = 11;
	rf.ch_min = 11;

	mask = ((uint32_t)(1 << (rf.ch_max + 1)) - (uint32_t)(1 << rf.ch_min));

	return mask;
}

static int sx1278_add_one(struct sx1278_phy *phy)
{
	struct ieee802154_hw *hw = phy->hw;
	int err;

	/* Define channels could be used. */
	hw->phy->supported.channels[0] = sx1278_ieee_channel_mask(hw);
	/* SX1278 phy channel 11 as default */
	hw->phy->current_channel = 11;
	phy->channel = hw->phy->current_channel;

	ieee802154_random_extended_addr(&hw->phy->perm_extended_addr);
	hw->flags = IEEE802154_HW_TX_OMIT_CKSUM
			| IEEE802154_HW_RX_OMIT_CKSUM
			| IEEE802154_HW_PROMISCUOUS;

	err = ieee802154_register_hw(hw);
	if (err)
		goto err_reg;

	INIT_WORK(&(phy->irqwork), sx1278_timer_irqwork);

	init_timer(&(phy->timer));
	phy->timer.expires = jiffies + HZ;
	phy->timer.function = sx1278_timer_isr;
	phy->timer.data = (unsigned long)phy;

	spin_lock_init(&(phy->buf_lock));

	err = init_sx127X(phy->rm);
	if (err)
		goto err_reg;

	return 0;

err_reg:
	dev_err(regmap_get_device(phy->rm),
		"register as IEEE 802.15.4 device failed\n");
	return err;
}

static void sx1278_del(struct sx1278_phy *phy)
{
	if (phy == NULL)
		return;

	del_timer(&(phy->timer));
	flush_work(&(phy->irqwork));

	ieee802154_unregister_hw(phy->hw);
	ieee802154_free_hw(phy->hw);
}

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
#define SX1278_ACPI_DUMMY	1
static const struct acpi_device_id sx1278_acpi_ids[] = {
	{ .id = "sx1278" },
	{},
};
MODULE_DEVICE_TABLE(acpi, sx1278_acpi_ids);

/* The callback function of ACPI probes SX1278 SPI. */
static void sx1278_probe_acpi(struct spi_device *spi) {
	const struct acpi_device_id *id;

	if (!has_acpi_companion(&(spi->dev)))
		return;

	id = acpi_match_device(sx1278_acpi_ids, &(spi->dev));
	if (WARN_ON(!id))
		return;

	if (id->driver_data == SX1278_ACPI_DUMMY)
		dev_warn(&(spi->dev),
			"Do not use this driver in produciton systems.\n");
}
#else
static void sx1278_probe_acpi(struct spi_device *spi) {};
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

static int sx1278_spi_probe(struct spi_device *spi)
{
	struct ieee802154_hw *hw;
	struct sx1278_phy *phy;
	int err;

#ifdef CONFIG_OF
	if (spi->dev.of_node && !of_match_device(sx1278_dt_ids, &(spi->dev))) {
		dev_err(&(spi->dev),
			"buggy DT: SX1278 listed directly in DT\n");
		WARN_ON(spi->dev.of_node &&
			!of_match_device(sx1278_dt_ids, &(spi->dev)));
	}
#endif
	sx1278_probe_acpi(spi);

	hw = ieee802154_alloc_hw(sizeof(*phy), &sx1278_ops);
	if (!hw) {
		dev_dbg(&(spi->dev), "not enough memory\n");
		return -ENOMEM;
	}

	phy = hw->priv;
	phy->hw = hw;
	phy->spi = spi;
	phy->rm = devm_regmap_init_spi(spi, &sx1278_regmap_config);
	hw->parent = &(spi->dev);

	/* Set the SPI device's driver data for later usage. */
	spi_set_drvdata(spi, phy);

	err = sx1278_add_one(phy);
	if (err < 0) {
		dev_err(&(spi->dev), "no SX1278 compatible device\n");
		goto err_slave;
	}

	dev_info(&(spi->dev), "add an IEEE 802.15.4 over LoRa SX1278 device\n");

	return 0;

err_slave:
	sx1278_del(phy);
	return err;
}

static int sx1278_spi_remove(struct spi_device *spi)
{
	struct sx1278_phy *phy = spi_get_drvdata(spi);

	sx1278_del(phy);

	return 0;
}

#define __DRIVER_NAME	"sx1278"

/* The SPI driver which acts as a protocol driver in this kernel module. */
static struct spi_driver sx1278_spi_driver = {
	.driver = {
		.name = __DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = sx1278_dt_ids,
#endif
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(sx1278_acpi_ids),
#endif
	},
	.probe = sx1278_spi_probe,
	.remove = sx1278_spi_remove,
	.id_table = spi_ids,
};

/* Register SX1278 kernel module. */
module_spi_driver(sx1278_spi_driver);

MODULE_AUTHOR("Jian-Hong Pan, <starnight@g.ncu.edu.tw>");
MODULE_DESCRIPTION("LoRa device SX1278 driver with SPI interface");
MODULE_LICENSE("Dual BSD/GPL");
