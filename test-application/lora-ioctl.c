#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>

#include "lora-ioctl.h"

/* Read the device data. */
ssize_t do_read(int fd, char *buf, size_t len) {
	ssize_t sz;

	memset(buf, '\0', 16);

	sz = read(fd, buf, 15);
	printf("Read %d bytes: %s\r\n", sz, buf);

	return sz;
}

/* Write data into the device. */
void do_write(int fd, char *buf, size_t len) {
	ssize_t sz;

	sz = write(fd, buf, strlen(buf));
	printf("Write %d bytes: %s\r\n", sz, buf);
}

/* Get current RSSI. */
int32_t get_rssi(int fd) {
	int32_t rssi;

	ioctl(fd, LORA_GET_RSSI, &rssi);

	return rssi;
}

/* Get last packet SNR. */
uint32_t get_snr(int fd) {
	uint32_t snr;

	ioctl(fd, LORA_GET_SNR, &snr);

	return snr;
}

/* Set & get output power. */
void set_power(int fd, int32_t power) {
	ioctl(fd, LORA_SET_POWER, &power);
}

int32_t get_power(int fd) {
	int32_t power;

	ioctl(fd, LORA_GET_POWER, &power);

	return power;
}

/* Set & get the RF spreading factor. */
void set_sprfactor(int fd, uint32_t sprf) {
	ioctl(fd, LORA_SET_SPRFACTOR, &sprf);
}

uint32_t get_sprfactor(int fd) {
	uint32_t sprf;

	ioctl(fd, LORA_GET_SPRFACTOR, &sprf);

	return sprf;
}
