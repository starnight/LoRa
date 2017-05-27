#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>

#include "lora-ioctl.h"

/* Read the device data. */
ssize_t do_read(int fd, char *buf, size_t len) {
	ssize_t sz;

	sz = read(fd, buf, len);

	return sz;
}

/* Write data into the device. */
ssize_t do_write(int fd, char *buf, size_t len) {
	ssize_t sz;

	sz = write(fd, buf, len);

	return sz;
}

/* Set the device in sleep state. */
void set_state(int fd, uint32_t st) {
	ioctl(fd, LORA_SET_STATE, &st);
}

uint32_t get_state(int fd) {
	uint32_t st;

	ioctl(fd, LORA_GET_STATE, &st);

	return st;
}

/* Set & get the carrier frequency. */
void set_freq(int fd, uint32_t freq) {
	ioctl(fd, LORA_SET_FREQUENCY, &freq);
}

uint32_t get_freq(int fd) {
	uint32_t freq;

	ioctl(fd, LORA_GET_FREQUENCY, &freq);

	return freq;
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

/* Set & get the RF bandwidth. */
void set_bw(int fd, uint32_t bw) {
	ioctl(fd, LORA_SET_BANDWIDTH, &bw);
}

uint32_t get_bw(int fd) {
	uint32_t bw;

	ioctl(fd, LORA_GET_BANDWIDTH, &bw);

	return bw;
}
