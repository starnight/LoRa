#ifndef __LORA_IOCTL_H__
#define __LORA_IOCTL_H__

#include <sys/ioctl.h>

/* I/O control by each command. */
#define LORA_IOC_MAGIC '\x66'

#define LORA_SET_STATE		(_IOW(LORA_IOC_MAGIC, 0, int))
#define LORA_GET_STATE		(_IOR(LORA_IOC_MAGIC, 1, int))
#define LORA_SET_FREQUENCY	(_IOW(LORA_IOC_MAGIC, 2, int))
#define LORA_GET_FREQUENCY	(_IOR(LORA_IOC_MAGIC, 3, int))
#define LORA_SET_POWER		(_IOW(LORA_IOC_MAGIC, 4, int))
#define LORA_GET_POWER		(_IOR(LORA_IOC_MAGIC, 5, int))
#define LORA_SET_BANDWIDTH	(_IOW(LORA_IOC_MAGIC, 6, int))
#define LORA_GET_BANDWIDTH	(_IOR(LORA_IOC_MAGIC, 7, int))


/* List the state of the LoRa device. */
#define LORA_STATE_SLEEP	0
#define LORA_STATE_STANDBY	1
#define LORA_STATE_TX		2
#define LORA_STATE_RX		3
#define LORA_STATE_CAD		4

#endif
