#ifndef __LORA_H__
#define __LORA_H__

#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/mutex.h>

/* List the state of the LoRa device. */
#define LORA_SLEEP		0
#define LORA_STANDBY	1
#define LORA_TX			2
#define LORA_RX			3

struct lora_data;

/* The structure lists the LoRa device's operations. */
struct lora_operations {
	/* Set & read the state of the LoRa device. */
	void (*setState)(uint8_t);
	uint8_t (*readState)(void);
	/* Set & get the carrier frequency. */
	void (*setFreq)(float);
	float (*getFreq)(void);
	/* Set & get the PA power. */
	void (*setPower)(uint32_t);
	uint32_t (*getPower)(void);
	/* Set & get the RF bandwith. */
	void (*setBW)(uint32_t);
	uint32_t (*getBW)(void);
	/* Read from the LoRa device's communication. */
	ssize_t (*read)(struct lora_data *, const char __user *, size_t);
	/* Write to the LoRa device's communication. */
	ssize_t (*write)(struct lora_data *, const char __user *, size_t);
};

struct lora_data {
	dev_t devt;
	void *lora_device;
	struct list_head device_entry;
	struct lora_operations *ops;
	uint8_t *tx_buf;
	uint8_t *rx_buf;
	uint8_t tx_buflen;
	uint8_t rx_buflen;
	uint8_t bufmaxlen;
	uint8_t users;
	struct mutex buf_lock;
	uint8_t node_adr;
	uint8_t packet_num;
};

struct lora_driver {
	char *name;
	int major;
	int minor_start;
	int num; // Max number of minor
	struct cdev lora_cdev;
	struct class *lora_class;
	struct module *owner;
};

#define MAX_PACKETLENGTH			123
#define LORA_OFFSET_PAYLOADLENGTH	5

struct lora_packet {
	uint8_t dst;
	uint8_t src;
	uint8_t packet_num;
	uint8_t len;
	uint8_t data[MAX_PACKETLENGTH];
	uint8_t retry;
};

#endif
