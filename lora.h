#ifndef __LoRa_H__
#define __LoRa_H__

#include <stdint.h>
#include <linux/fs.h>

/* List the state of the LoRa device. */
#define LORA_SLEEP		0
#define LORA_STANDBY	1
#define LORA_TX			2
#define LORA_RX			3

/* The structure lists the LoRa device's operations. */
struct lora_operations {
	/* Set & read the state of the LoRa device. */
	void setState(uint8_t);
	uint8_t readState(void);
	/* Set & get the carrier frequency. */
	void setFreq(float);
	float getFreq(void);
	/* Set & get the PA power. */
	void setPower(uint32_t);
	uint32_t getPower(void);
	/* Set & get the RF bandwith. */
	void setBW(uint32_t);
	uint32_t getBW(void);
	/* Read from the LoRa device's communication. */
	size_t read(struct file *, uint8_t *, size_t);
	/* Write to the LoRa device's communication. */
	size_t write(struct file *, uint8_t *, size_t);
};

#endif
