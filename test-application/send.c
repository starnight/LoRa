#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/ioctl.h>

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

int main(int argc, char **argv) {
	char *path;
	char *data;
	int fd;
	char pstr[40];
#define MAX_BUFFER_LEN	16
	char buf[MAX_BUFFER_LEN];
	int len;
	uint32_t frq;

	/* Parse command. */
	if(argc >= 3) {
		path = argv[1];
		data = argv[2];
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

	/* Set the carrier frequency. */
	frq = 433000000;
	ioctl(fd, LORA_SET_FREQUENCY, &frq);
	printf("Set he LoRa carrier frequency to be %u Hz\n", frq);

	/* Write to the file descriptor if it is ready to be written. */
	printf("Going to write %s\n", path);
	do_write(fd, data, strlen(data));
	printf("Written %s\n", path);

	/* Read from echo if it is ready to be read. */
	memset(buf, 0, MAX_BUFFER_LEN);
	printf("Going to read %s\n", path);
	len = do_read(fd, buf, MAX_BUFFER_LEN);
	if(len > 0)
		printf("Read %s\n", path);

	/* Get the carrier frequency. */
	ioctl(fd, LORA_GET_FREQUENCY, &frq);
	printf("The LoRa carrier frequency is %u Hz\n", frq);

	/* Close device node. */
	close(fd);

	return 0;
}
