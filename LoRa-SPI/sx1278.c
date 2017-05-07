#include <linux/init.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#include "sx1278.h"

#define F_XOSC		32000000
#define	POW_2_19	0x80000

int init_sx127X(struct spi_device *spi) {
	char ver[5];
	uint8_t implicitheader = 0x73;
	printk(KERN_DEBUG "sx127X: init sx127X\n");

	sx127X_readVersion(spi, ver, 4);
	ver[4] = '\0';
	printk(KERN_DEBUG "sx127X: chip version %s\n", ver);

	sx127X_startLoRaMode(spi);
	sx127X_write_reg(spi, SX127X_REG_MODEM_CONFIG1, &implicitheader, 1);

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

ssize_t
sx127X_sync_write(struct spi_device *spi, uint8_t *buf, size_t len) {
	struct spi_transfer t;
	struct spi_message m;

	spi_message_init(&m);
	memset(&t, 0, sizeof(t));
	t.tx_buf = buf;
	t.len = len;

	spi_message_add_tail(&t, &m);
	return spi_sync(spi, &m);
}

ssize_t
sx127X_sync_read(struct spi_device *spi, uint8_t *buf, size_t len) {
	struct spi_transfer t = {
		.rx_buf = buf,
		.len = len,
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return sx127X_sync(spi, &m);
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
	spi_message_add_tail(&at, &m);

	/* Write value. */
	memset(&bt, 0, sizeof(bt));
	bt.rx_buf = buf;
	bt.len = len;
	spi_message_add_tail(&bt, &m);

	status = sx127X_sync(spi, &m);
	/* Minus the start address's length. */
	if(status > 0)
		status -= 1;
	//printk(KERN_DEBUG "sx127X: just read %d bytes from chip\n", status);

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
	spi_message_add_tail(&at, &m);

	/* Write value. */
	memset(&bt, 0, sizeof(bt));
	bt.tx_buf = buf;
	bt.len = len;
	spi_message_add_tail(&bt, &m);

	status = sx127X_sync(spi, &m);
	/* Minus the start address's length. */
	if(status > 0)
		status -= 1;
	//printk(KERN_DEBUG "sx127X: just written %d bytes to chip\n", status);

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

void
sx127X_setPower(struct spi_device *spi, uint32_t mW) {

}
//uint32_t sx127X_getPower(struct spi_device *spi) {}

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
sx127X_getLoRaFlag(struct spi_device *spi, uint8_t f) {
	uint8_t flags;

	sx127X_read_reg(spi, SX127X_REG_IRQ_FLAGS, &flags, 1);

	return (flags & f);
}

void
sx127X_clearLoRaFlag(struct spi_device *spi, uint8_t f) {
	uint8_t flag;

	/* Get oiginal flag. */
	flag = sx127X_getAllLoRaFlag(spi);
	/* Set the designated bits of the flag. */
	flag |= f;
	sx127X_write_reg(spi, SX127X_REG_IRQ_FLAGS, &flag, 1);
}
//uint8_t sx127X_getLoRaRXEndFlag(void) {}
//ssize_t sz127X_setLoRaDataToSend(uint8_t *buf, size_t len) {}
//void sx127X_setLoRaSPRFactor(uint8_t spf) {}
uint32_t
sx127X_getLoRaSPRFactor(struct spi_device *spi) {
	uint8_t sf;
	uint32_t chips;

	sx127X_read_reg(spi, SX127X_REG_MODEM_CONFIG2, &sf, 1);
	sf = sf >> 4;
	chips = 1 << sf;

	return chips;
}

//void sx127X_setLoRaBW(struct spi_device *spi, uint32_t bw) {}

uint32_t
sx127X_getLoRaBW(struct spi_device *spi) {
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
	uint8_t bw;

	sx127X_read_reg(spi, SX127X_REG_MODEM_CONFIG1, &bw, 1);
	bw = bw >> 4;

	return hz[bw];
}
//void sx127X_setLoRaCR(uint8_t cr) {}
//uint8_t sx127X_getLoRaCR(void) {}
//float sx127X_getSRate(void) {}
//void sx127X_setLoRaRXByteTimeout(uint32_t n) {}
//void sx127X_setLoRaRXTimeout(float sec) {}
//uint32_t sx127X_getLoRaRXByteTimeout(void) {}
//float sx127X_getLoRaRXTimeout(void) {}
//void sx127X_setLoRaMaxRXBuff(uint8_t len) {}
//size_t sx127X_readLoRaData(uint8_t *buf, size_t len) {}
//void sx127X_DiscardLoRaRX(void) {}
//int32_t sx127X_LastLoRaPacketRSSI(void) {}
//uint32_t sx127X_LastLoRaPacketSNR(void) {}
//int32_t sx127X_getLoRaRSSI(void) {}
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
//void sx127X_setLoRaCRC(uint8_t yesno) {}
//void sx127X_setBoost(uint8_t yesno) {}
