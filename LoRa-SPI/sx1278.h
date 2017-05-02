#ifndef __SX1278_H__
#define __SX1278_H__

/* SX127x Registers addresses */
#define SX127X_REG_FIFO						0x00
#define SX127X_REG_OP_MODE					0x01
#define SX127X_REG_FRF_MSB					0x06
#define SX127X_REG_FRF_MID					0x07
#define SX127X_REG_FRF_LSB					0x08
#define SX127X_REG_PA_CONFIG				0x09
#define SX127X_REG_PA_RAMP					0x0A
#define SX127X_REG_OCP						0x0B
#define SX127X_REG_LNA						0x0C
#define SX127X_REG_FIFO_ADDR_PTR			0x0D
#define SX127X_REG_FIFO_TX_BASE_ADDR		0x0E
#define SX127X_REG_FIFO_RX_BASE_ADDR		0x0F
#define SX127X_REG_FIFO_RX_CURRENT_ADDR		0x10
#define SX127X_REG_IRQ_FLAGS_MASK			0x11
#define SX127X_REG_IRQ_FLAGS				0x12
#define SX127X_REG_RX_NB_BYTES				0x13
#define SX127X_REG_RX_HEADER_CNT_VALUE_MSB	0x14
#define SX127X_REG_RX_HEADER_CNT_VALUE_LSB	0x15
#define SX127X_REG_RX_PACKET_CNT_VALUE_MSB	0x16
#define SX127X_REG_RX_PACKET_CNT_VALUE_LSB	0x17
#define SX127X_REG_MODEM_STAT				0x18
#define SX127X_REG_PKT_SNR_VALUE			0x19
#define SX127X_REG_PKT_RSSI_VALUE			0x1A
#define SX127X_REG_RSSI_VALUE				0x1B
#define SX127X_REG_HOP_CHANNEL				0x1C
#define SX127X_REG_MODEM_CONFIG1			0x1D
#define SX127X_REG_MODEM_CONFIG2			0x1E
#define SX127X_REG_SYMB_TIMEOUT_LSB			0x1F
#define SX127X_REG_PREAMBLE_MSB				0x20
#define SX127X_REG_PREAMBLE_LSB				0x21
#define SX127X_REG_PAYLOAD_LENGTH			0x22
#define SX127X_REG_MAX_PAYLOAD_LENGTH		0x23
#define SX127X_REG_HOP_PERIOD				0x24
#define SX127X_REG_FIFO_RX_BYTE_ADDR		0x25
#define SX127X_REG_MODEM_CONFIG3			0x26
#define SX127X_REG_FEI_MSB					0x28
#define SX127X_REG_FEI_MID					0x29
#define SX127X_REG_FEI_LSB					0x2A
#define SX127X_REG_RSSI_WIDEBAND			0x2C
#define SX127X_REG_DETECT_OPTIMIZE			0x31
#define SX127X_REG_INVERT_IRQ				0x33
#define SX127X_REG_DETECTION_THRESHOLD		0x37
#define SX127X_REG_SYNC_WORD				0x39
#define SX127X_REG_VERSION					0x42
#define SX127X_REG_TCXO						0x4B
#define SX127X_REG_PA_DAC					0x4D
#define SX127X_REG_FORMER_TEMP				0x5B
#define SX127X_REG_AGC_REF					0x61
#define SX127X_REG_AGC_THRESH1				0x62
#define SX127X_REG_AGC_THRESH2				0x63
#define SX127X_REG_AGC_THRESH3				0x64
#define SX127X_REG_PLL						0x70

#define SX127X_SLEEP_MODE					0x00
#define SX127x_STANDBY_MODE					0x01
#define SX127x_FSTX_MODE					0x02
#define SX127x_TX_MODE						0x03
#define SX127x_FSRX_MODE					0x04
#define SX127X_RX_MODE						0x05

int init_sx1278(struct spi_device *spi);

ssize_t
sx1278_sync(struct spi_device *spi, struct spi_message *);

ssize_t
sx1278_sync_write(struct spi_device *spi, uint8_t *buf, size_t len);

ssize_t
sx1278_sync_read(struct spi_device *spi, uint8_t *buf, size_t len);

int
sx1278_read_reg(struct spi_device *spi, uint8_t start_adr, uint8_t *buf, size_t len);

int
sx1278_write_reg(struct spi_device *spi, uint8_t start_adr, uint8_t *buf, size_t len);

void
sx127X_restart(struct spi_device *spi);

void
sx127X_startLoRaMode(struct spi_device *spi);

uint8_t 
sx127X_readMode(struct spi_device *spi);

void
sx127X_setState(struct spi_device *spi, uint8_t st);

uint8_t
sx127X_readState(struct spi_device *spi);



int
sx127X_readVersion(struct spi_device *spi, char *vstr, size_t len);

#endif
