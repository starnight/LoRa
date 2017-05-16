#include <linux/init.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#include "sx1278.h"

#define F_XOSC		32000000
#define	POW_2_19	0x80000

int init_sx127X(struct spi_device *spi) {
	char ver[5];
	printk(KERN_DEBUG "sx127X: init sx127X\n");

	sx127X_readVersion(spi, ver, 4);
	ver[4] = '\0';
	printk(KERN_DEBUG "sx127X: chip version %s\n", ver);

	sx127X_startLoRaMode(spi);

	return 0;
}

ssize_t
sx127X_sync(struct spi_device *spi, struct spi_message *m) {
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	if(spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_sync(spi, m);

	if(status == 0)
		status = m->actual_length;

	return status;
}

int
sx127X_read_reg(struct spi_device *spi, uint8_t start_adr, uint8_t *buf, size_t len) {
	int status = 0;
	struct spi_transfer at, bt;
	struct spi_message m;

	spi_message_init(&m);

	/* Read address.  The MSB must be cleared because of reading an address. */
	memset(&at, 0, sizeof(at));
	at.tx_buf = &start_adr;
	at.len = 1;
	at.speed_hz = 15200;
	spi_message_add_tail(&at, &m);

	/* Read value. */
	memset(&bt, 0, sizeof(bt));
	bt.rx_buf = buf;
	bt.len = len;
	bt.speed_hz = 15200;
	spi_message_add_tail(&bt, &m);

	status = sx127X_sync(spi, &m);
	/* Minus the start address's length. */
	if(status > 0)
		status -= 1;

	return status;
}

int
sx127X_write_reg(struct spi_device *spi, uint8_t start_adr, uint8_t *buf, size_t len) {
	int status = 0;
	struct spi_transfer at, bt;
	struct spi_message m;

	spi_message_init(&m);

	/* Write address.  The MSB must be set because of writing an address. */
	start_adr |= 0x80;
	memset(&at, 0, sizeof(at));
	at.tx_buf = &start_adr;
	at.len = 1;
	at.speed_hz = 15200;
	spi_message_add_tail(&at, &m);

	/* Write value. */
	memset(&bt, 0, sizeof(bt));
	bt.tx_buf = buf;
	bt.len = len;
	bt.speed_hz = 15200;
	spi_message_add_tail(&bt, &m);

	status = sx127X_sync(spi, &m);
	/* Minus the start address's length. */
	if(status > 0)
		status -= 1;

	return status;
}

void
sx127X_restart(struct spi_device *spi) {}

void
sx127X_startLoRaMode(struct spi_device *spi) {
	uint8_t op_mode;

	/* Get original OP Mode register. */
	op_mode = sx127X_readMode(spi);
	printk(KERN_DEBUG "sx127X: the original OP mode is 0x%X\n", op_mode);
	/* Set device to sleep state. */
	sx127X_setState(spi, SX127X_SLEEP_MODE);
	/* Set device to LoRa mode. */
	op_mode = sx127X_readMode(spi);
	op_mode = op_mode | 0x80;
	sx127X_write_reg(spi, SX127X_REG_OP_MODE, &op_mode, 1);
	/* Set device to standby state. */
	sx127X_setState(spi, SX127X_STANDBY_MODE);
	op_mode = sx127X_readMode(spi);
	printk(KERN_DEBUG "sx127X: the current OP mode is 0x%X\n", op_mode);
}

uint8_t 
sx127X_readMode(struct spi_device *spi) {
	uint8_t op_mode;

	/* Get original OP Mode register. */
	sx127X_read_reg(spi, SX127X_REG_OP_MODE, &op_mode, 1);

	return op_mode;
}

void
sx127X_setState(struct spi_device *spi, uint8_t st) {
	uint8_t op_mode;

	/* Get original OP Mode register. */
	op_mode = sx127X_readMode(spi);
	/* Set device to designated state. */
	op_mode = (op_mode & 0xF8) | (st & 0x07);
	sx127X_write_reg(spi, SX127X_REG_OP_MODE, &op_mode, 1);
}

uint8_t
sx127X_readState(struct spi_device *spi) {
	uint8_t op_mode;

	op_mode = sx127X_readMode(spi);

	return op_mode & 0x07;
}

void
sx127X_setFreq(struct spi_device *spi, uint32_t fr) {
	uint32_t frt = 0;
	uint8_t buf[3];
	int i;

	frt = fr * POW_2_19 / F_XOSC;

	for(i = 2; i <= 0; i--) {
		buf[i] = frt % 256;
		frt = frt >> 8;
	}

	sx127X_write_reg(spi, SX127X_REG_FRF_MSB, buf, 3);
}

uint32_t
sx127X_getFreq(struct spi_device *spi) {
	uint32_t frt = 0;
	uint8_t buf[3];
	int status;
	int i;
	uint32_t fr;

	status = sx127X_read_reg(spi, SX127X_REG_FRF_MSB, buf, 3);
	if(status <= 0)
		return 0.0;

	for(i = 2; i >= 0; i--)
		frt = frt * 256 + buf[i];

	fr =  frt * F_XOSC / POW_2_19;

	return fr;
}

//void sx127X_setPower(struct spi_device *spi, uint32_t mW) {}

uint8_t
sx127X_getPower(struct spi_device *spi) {
	uint8_t pac;
	uint8_t boost;
	uint8_t pmax;
	uint8_t pout;
	uint8_t dbm;

	sx127X_read_reg(spi, SX127X_REG_PA_CONFIG, &pac, 1);
	boost = (pac & 0x80) >> 7;
	pout = pac & 0x0F;
	if(boost) {
		dbm = pout - 2;
	}
	else {
		/* Power max should be pmax/10.  Itis 10 times for now. */
		pmax = (108 + 6 * ((pac & 0x70) >> 4));
		dbm = (150 - pmax + pout * 10) / 10;
	}

	return dbm;
}

int
sx127X_readVersion(struct spi_device *spi, char *vstr, size_t len) {
	uint8_t v;
	uint8_t fv, mmv;
	int status;

	status = sx127X_read_reg(spi, SX127X_REG_VERSION, &v, 1);

	if(status == 1) {
		fv = v >> 4;
		mmv = v & 0x0F;
		snprintf(vstr, len, "%d.%d", fv, mmv);
	}

	return status;
}

/* LoRa Mode */
uint8_t
sx127X_getLoRaAllFlag(struct spi_device *spi) {
	uint8_t flags;

	sx127X_read_reg(spi, SX127X_REG_IRQ_FLAGS, &flags, 1);

	return flags;
}

void
sx127X_clearLoRaFlag(struct spi_device *spi, uint8_t f) {
	uint8_t flag;

	/* Get oiginal flag. */
	flag = sx127X_getLoRaAllFlag(spi);
	/* Set the designated bits of the flag. */
	flag |= f;
	sx127X_write_reg(spi, SX127X_REG_IRQ_FLAGS, &flag, 1);
}
//uint8_t sx127X_getLoRaRXEndFlag(struct spi_device *spi) {}
//ssize_t sz127X_setLoRaDataToSend(struct spi_device *spi, uint8_t *buf, size_t len) {}

void
sx127X_setLoRaSPRFactor(struct spi_device *spi, uint32_t chips) {
	uint8_t sf;
	uint8_t mcf2;

	for(sf = 6; sf < 12; sf++) {
		if(chips == ((uint32_t)1 << sf))
			break;
	}

	sx127X_read_reg(spi, SX127X_REG_MODEM_CONFIG2, &mcf2, 1);
	mcf2 = (mcf2 & 0x0F) | (sf << 4);
	sx127X_write_reg(spi, SX127X_REG_MODEM_CONFIG2, &mcf2, 1);
}

uint32_t
sx127X_getLoRaSPRFactor(struct spi_device *spi) {
	uint8_t sf;
	uint32_t chips;

	sx127X_read_reg(spi, SX127X_REG_MODEM_CONFIG2, &sf, 1);
	sf = sf >> 4;
	chips = 1 << sf;

	return chips;
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

void
sx127X_setLoRaBW(struct spi_device *spi, uint32_t bw) {
	uint8_t i;
	uint8_t mcf1;

	for(i = 0; i < 9; i++) {
		if(hz[i] <= bw)
			break;
	}

	sx127X_read_reg(spi, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
	mcf1 = (mcf1 & 0x0F) | i;
	sx127X_write_reg(spi, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
}

uint32_t
sx127X_getLoRaBW(struct spi_device *spi) {
	uint8_t mcf1;
	uint8_t bw;

	sx127X_read_reg(spi, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
	bw = mcf1 >> 4;

	return hz[bw];
}

void
sx127X_setLoRaCR(struct spi_device *spi, uint8_t cr) {
	uint8_t mcf1;

	sx127X_read_reg(spi, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
	mcf1 = (mcf1 & 0x0E) | (((cr & 0xF) - 4) << 1);
	sx127X_write_reg(spi, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
}

uint8_t
sx127X_getLoRaCR(struct spi_device *spi) {
	uint8_t mcf1;
	uint8_t cr; // ex: 0x45 represents cr=4/5

	sx127X_read_reg(spi, SX127X_REG_MODEM_CONFIG1, &mcf1, 1);
	cr = 0x40 + ((mcf1 & 0x0E) >> 1) + 4;
	
	return cr;
}
//float sx127X_getSRate(struct spi_device *spi) {}
void
sx127X_setLoRaRXByteTimeout(struct spi_device *spi, uint32_t n) {
	uint8_t buf[2];
	uint8_t mcf2;

	if(n < 1) n = 1;
	if(n > 1023) n = 1023;

	/* Read original Modem config 2. */
	sx127X_read_reg(spi, SX127X_REG_MODEM_CONFIG2, &mcf2, 1);

	/* LSB */
	buf[1] = n % 256;
	/* MSB */
	buf[0] = (mcf2 & 0xFC) | (n >> 8);

	sx127X_write_reg(spi, SX127X_REG_MODEM_CONFIG2, buf, 2);
}

void
sx127X_setLoRaRXTimeout(struct spi_device *spi, uint32_t ms) {
	uint32_t n;

	n = ms * sx127X_getLoRaBW(spi) / sx127X_getLoRaSPRFactor(spi);

	sx127X_setLoRaRXByteTimeout(spi, n);
}

uint32_t
sx127X_getLoRaRXByteTimeout(struct spi_device *spi) {
	uint32_t n;
	uint8_t buf[2];

	sx127X_read_reg(spi, SX127X_REG_MODEM_CONFIG2, buf, 2);

	n = (buf[0] & 0x03) * 256 + buf[1];

	return n;
}

uint32_t
sx127X_getLoRaRXTimeout(struct spi_device *spi) {
	uint32_t ms;

	ms = sx127X_getLoRaRXByteTimeout(spi) * sx127X_getLoRaSPRFactor(spi) / sx127X_getLoRaBW(spi);

	return ms;
}

void
sx127X_setLoRaMaxRXBuff(struct spi_device *spi, uint8_t len) {
	sx127X_write_reg(spi, SX127X_REG_MAX_PAYLOAD_LENGTH, &len, 1);
}

ssize_t
sx127X_readLoRaData(struct spi_device *spi, uint8_t *buf, size_t len) {
	uint8_t start_adr;
	uint8_t blen;
	ssize_t c;

	/* Get the chip RX FIFO last packet address. */
	sx127X_read_reg(spi, SX127X_REG_FIFO_RX_CURRENT_ADDR, &start_adr, 1);
	/* Set chip FIFO pointer to FIFO last packet address. */
	sx127X_write_reg(spi, SX127X_REG_FIFO_ADDR_PTR, &start_adr, 1);

	/* Get the RX last packet payload length. */
	sx127X_read_reg(spi, SX127X_REG_RX_NB_BYTES, &blen, 1);

	len = (blen < len) ? blen : len;
	//printk(KERN_DEBUG "sx127X: going to read %d bytes from RX FIFO\n", len);
	/* Read LoRa packet payload. */
	c = sx127X_read_reg(spi, SX127X_REG_FIFO, buf, len);
	//printk(KERN_DEBUG "sx127X: c in read LoRa data is %d\n", c);

	return c;
}

//void sx127X_discardLoRaRX(struct spi_device *spi) {}
int32_t
sx127X_getLastLoRaPacketRSSI(struct spi_device *spi) {
	uint32_t dbm;
	uint8_t lhf;
	uint8_t rssi;

	/* Get LoRa is in high or low frequency mode. */
	lhf = sx127X_readMode(spi) & 0x08;
	/* Get RSSI value. */
	sx127X_read_reg(spi, SX127X_REG_PKT_RSSI_VALUE, &rssi, 1);
	dbm = (lhf) ? -164 + rssi : -157 + rssi;

	return dbm;
}

uint32_t
sx127X_getLastLoRaPacketSNR(struct spi_device *spi) {
	uint32_t db;
	uint8_t snr;

	sx127X_read_reg(spi, SX127X_REG_PKT_SNR_VALUE, &snr, 1);
	db = snr / 4;

	return db;
}

int32_t
sx127X_getLoRaRSSI(struct spi_device *spi) {
	uint32_t dbm;
	uint8_t lhf;
	uint8_t rssi;

	/* Get LoRa is in high or low frequency mode. */
	lhf = sx127X_readMode(spi) & 0x08;
	/* Get RSSI value. */
	sx127X_read_reg(spi, SX127X_REG_RSSI_VALUE, &rssi, 1);
	dbm = (lhf) ? -164 + rssi : -157 + rssi;

	return dbm;
}

void
sx127X_setLoRaPreambleLen(struct spi_device *spi, uint32_t len) {
	uint8_t pl[2];

	pl[1] = len % 256;
	pl[0] = (len >> 8) % 256;

	sx127X_write_reg(spi, SX127X_REG_PREAMBLE_MSB, pl, 2);
}

uint32_t
sx127X_getLoRaPreambleLen(struct spi_device *spi) {
	uint8_t pl[2];
	uint32_t len;

	sx127X_read_reg(spi, SX127X_REG_PREAMBLE_MSB, pl, 2);
	len = pl[0] * 256 + pl[1];

	return len;
}

void
sx127X_setLoRaCRC(struct spi_device *spi, uint8_t yesno) {
	uint8_t mcf2;

	sx127X_read_reg(spi, SX127X_REG_MODEM_CONFIG2, &mcf2, 1);
	mcf2 = (yesno) ? mcf2 | (1 << 2) : mcf2 & (~(1 << 2));
	sx127X_write_reg(spi, SX127X_REG_MODEM_CONFIG2, &mcf2, 1);
}

void
sx127X_setBoost(struct spi_device *spi, uint8_t yesno) {
	uint8_t pacf;

	sx127X_read_reg(spi, SX127X_REG_PA_CONFIG, &pacf, 1);
	pacf = (yesno) ? pacf | (1 << 7) : pacf & (~(1 << 7));
	sx127X_write_reg(spi, SX127X_REG_PA_CONFIG, &pacf, 1);
}
