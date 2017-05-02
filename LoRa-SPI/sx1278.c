#include <linux/init.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#include "sx1278.h"

#define F_XOSC		32000000
#define	POW_2_19	0x80000

int init_sx1278(struct spi_device *spi) {
	char ver[5];
	printk(KERN_DEBUG "sx1278: init sx1278\n");
	
	sx127X_setState(spi, SX127X_SLEEP_MODE);
	sx127X_readVersion(spi, ver, 4);

	ver[4] = '\0';
	printk(KERN_DEBUG "sx1278: chip version %s\n", ver);

	return 0;
}

ssize_t
sx1278_sync(struct spi_device *spi, struct spi_message *m) {
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
sx1278_sync_write(struct spi_device *spi, uint8_t *buf, size_t len) {
	struct spi_transfer t = {
		.tx_buf = buf,
		.len = len,
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return sx1278_sync(spi, &m);
}

ssize_t
sx1278_sync_read(struct spi_device *spi, uint8_t * buf, size_t len) {
	struct spi_transfer t = {
		.rx_buf = buf,
		.len = len,
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return sx1278_sync(spi, &m);
}

int
sx1278_read_reg(struct spi_device *spi, uint8_t start_adr, uint8_t *buf, size_t len) {
	return spi_write_then_read(spi, &start_adr, 1, buf, len);
}

int
sx1278_write_reg(struct spi_device *spi, uint8_t start_adr, uint8_t *buf, size_t len) {
	int status;

	status = sx1278_sync_write(spi, &start_adr, 1);
	if(status > 0)
		status = sx1278_sync_write(spi, buf, len);

	return status;
}

void
sx127X_restart(struct spi_device *spi) {}

void
sx127X_startLoRaMode(struct spi_device *spi) {
	uint8_t op_mode;

	/* Set device to sleep state. */
	sx127X_setState(spi, SX127X_SLEEP_MODE);
	/* Get original OP Mode register. */
	op_mode = sx127X_readMode(spi);
	/* Set device to LoRa mode. */
	op_mode = (op_mode & 0x7F) | 0x80;
	sx1278_write_reg(spi, SX127X_REG_OP_MODE, &op_mode, 1);
}

uint8_t 
sx127X_readMode(struct spi_device *spi) {
	uint8_t op_mode;

	/* Get original OP Mode register. */
	sx1278_read_reg(spi, SX127X_REG_OP_MODE, &op_mode, 1);

	return op_mode;
}

void
sx127X_setState(struct spi_device *spi, uint8_t st) {
	uint8_t op_mode;

	/* Get original OP Mode register. */
	op_mode = sx127X_readMode(spi);
	/* Set device to designated state. */
	op_mode = (op_mode & 0xF8) | (st & 0x07);
	sx1278_write_reg(spi, SX127X_REG_OP_MODE, &op_mode, 1);
}

uint8_t
sx127X_readState(struct spi_device *spi) {
	uint8_t op_mode;
	
	op_mode = sx127X_readMode(spi);

	return op_mode & 0x07;
}

void sx127X_setFreq(struct spi_device *spi, uint32_t fr) {
	uint32_t frt = 0;
	uint8_t buf[3];
	int i;

	frt = fr * POW_2_19 / F_XOSC;

	for(i = 2; i <= 0; i--) {
		buf[i] = frt % 256;
		frt = frt >> 8;
	}

	sx1278_write_reg(spi, SX127X_REG_FRF_MSB, buf, 3);
}

uint32_t sx127X_getFreq(struct spi_device *spi) {
	uint32_t frt = 0;
	uint8_t buf[3];
	int status;
	int i;
	uint32_t fr;

	status = sx1278_read_reg(spi, SX127X_REG_FRF_MSB, buf, 3);
	if(status != 0)
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

	status = sx1278_read_reg(spi, SX127X_REG_VERSION, &v, 1);

	if(status == 0) {
		fv = v >> 4;
		mmv = v & 0x0F;
		snprintf(vstr, len, "%d.%d", fv, mmv);
	}

	return status;
}

/* LoRa Mode */
//uint8_t sx127X_getLoRaFlag(void) {}
//void sx127X_clearLoRaFlag(uint8_t f) {}
//void sx127x_clearAllLoRaFlag(void) {}
//uint8_t sx127X_getLoRaRXEndFlag(void) {}
//ssize_t sz127X_setLoRaDataToSend(uint8_t *buf, size_t len) {}
//void sx127X_setLoRaSPRFactor(uint8_t spf) {}
//uint8_t sx127X_getLoRaSPRFactor(void) {}
//void sx127X_setLoRaBW(uint32_t bw) {}
//uint32_t sx127X_getLoRaBW(void) {}
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
//void sx127X_setLoRaPreambleLen(uint32_t len) {}
//uint32_t sx127X_getLoRaPreambleLen(void) {}
//void sx127X_setLoRaCRC(uint8_t yesno) {}
//void sx127X_setBoost(uint8_t yesno) {}
