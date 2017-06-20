/*-
 * Copyright (c) 2017 Jian-Hong, Pan <starnight@g.ncu.edu.tw>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 *
 */

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
