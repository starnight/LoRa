#ifndef __LORA_IOCTL_H__
#define __LORA_IOCTL_H__

#include <sys/ioctl.h>

/* I/O control by each command. */
#define LORA_IOC_MAGIC '\x66'

#define LORA_SET_STATE		(_IOW(LORA_IOC_MAGIC,  0, int))
#define LORA_GET_STATE		(_IOR(LORA_IOC_MAGIC,  1, int))
#define LORA_SET_FREQUENCY	(_IOW(LORA_IOC_MAGIC,  2, int))
#define LORA_GET_FREQUENCY	(_IOR(LORA_IOC_MAGIC,  3, int))
#define LORA_SET_POWER		(_IOW(LORA_IOC_MAGIC,  4, int))
#define LORA_GET_POWER		(_IOR(LORA_IOC_MAGIC,  5, int))
#define LORA_SET_SPRFACTOR	(_IOW(LORA_IOC_MAGIC,  6, int))
#define LORA_GET_SPRFACTOR	(_IOR(LORA_IOC_MAGIC,  7, int))
#define LORA_SET_BANDWIDTH	(_IOW(LORA_IOC_MAGIC,  8, int))
#define LORA_GET_BANDWIDTH	(_IOR(LORA_IOC_MAGIC,  9, int))
#define LORA_GET_RSSI		(_IOR(LORA_IOC_MAGIC, 10, int))
#define LORA_GET_SNR		(_IOR(LORA_IOC_MAGIC, 11, int))

/* List the state of the LoRa device. */
#define LORA_STATE_SLEEP	0
#define LORA_STATE_STANDBY	1
#define LORA_STATE_TX		2
#define LORA_STATE_RX		3
#define LORA_STATE_CAD		4

/* Read the device data. */
ssize_t do_read(int fd, char *buf, size_t len);

/* Write data into the device. */
ssize_t do_write(int fd, char *buf, size_t len);

/* Set the device in sleep state. */
void set_state(int fd, uint32_t st);
uint32_t get_state(int fd);

/* Set & get the carrier frequency. */
void set_freq(int fd, uint32_t freq);
uint32_t get_freq(int fd);

/* Get current RSSI. */
int32_t get_rssi(int fd);

/* Get last packet SNR. */
uint32_t get_snr(int fd);

/* Set & get output power. */
void set_power(int fd, int32_t power);
int32_t get_power(int fd);

/* Set & get the RF spreading factor. */
void set_sprfactor(int fd, uint32_t sprf);
uint32_t get_sprfactor(int fd);

/* Set & get the RF bandwidth. */
void set_bw(int fd, uint32_t bw);
uint32_t get_bw(int fd);

#endif
