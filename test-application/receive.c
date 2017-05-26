#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <ctype.h>

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

/* Get output power. */
uint32_t get_power(int fd) {
	uint32_t power;

	ioctl(fd, LORA_GET_POWER, &power);

	return power;
}

int main(int argc, char **argv) {
	char *path;
	int fd;
	char pstr[40];
#define MAX_BUFFER_LEN	16
	char buf[MAX_BUFFER_LEN];
	int len;
	int i;

	/* Parse command. */
	if(argc >= 2) {
		path = argv[1];
	}
	else {
		printf("Need more arguments.\r\n");
		return -1;
	}

	printf("Going to open %s\n", path);
	/* Open device node. */
	fd = open(path, O_RDWR);
	printf("Opened %s\n", path);
	if(fd == -1) {
		sprintf(pstr, "Open %s failed", path);
		perror(pstr);
		return -1;
	}

	/* Set the RF bandwidth. */
	uint32_t bw = 125000;
	ioctl(fd, LORA_SET_BANDWIDTH, &bw);
	printf("Going to set the RF bandwith %u Hz\n", bw);

	printf("The current RSSI is %d dbm\n", get_rssi(fd));

	/* Read from the file descriptor if it is ready to be read. */
	memset(buf, 0, MAX_BUFFER_LEN);
	printf("Going to read %s\n", path);
	len = do_read(fd, buf, MAX_BUFFER_LEN);

	if(len > 0) {
		printf("Read %s\n", path);

		sleep(2);

		/* Echo */
		printf("Going to echo %s\n", path);
		for(i = 0; i < len; i++)
			buf[i] = toupper(buf[i]);
		do_write(fd, buf, len);
		printf("Echoed %s\n", path);
	}

	/* Get the carrier frequency. */
	uint32_t frq;
	ioctl(fd, LORA_GET_FREQUENCY, &frq);
	printf("The LoRa carrier frequency is %u Hz\n", frq);

	/* Get the RF bandwidth. */
	bw = 0;
	ioctl(fd, LORA_GET_BANDWIDTH, &bw);
	printf("The RF bandwith is %u Hz\n", bw);

	printf("The current RSSI is %d dbm\n", get_rssi(fd));
	printf("The last packet SNR is %u db\n", get_snr(fd));
	printf("The output power is %u dbm\n", get_power(fd));

	/* Set the device in sleep state. */
	uint32_t st = LORA_STATE_SLEEP;
	ioctl(fd, LORA_SET_STATE, &st);
	st = 0xFF;
	ioctl(fd, LORA_GET_STATE, &st);
	printf("The LoRa device is in 0x%X state\n", st);

	/* Close device node. */
	close(fd);

	return 0;
}
