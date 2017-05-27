#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/ioctl.h>

#include "lora-ioctl.h"

int main(int argc, char **argv) {
	char *path;
	char *data;
	int fd;
	char pstr[40];
#define MAX_BUFFER_LEN	16
	char buf[MAX_BUFFER_LEN];
	int len;

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

	/*Set the RF spreading factor. */
	uint32_t sprf = 2048;
	set_sprfactor(fd, sprf);
	printf("Going to set the RF spreading factor %u chips\n", sprf);

	/* Set the RF bandwidth. */
	uint32_t bw = 125000;
	ioctl(fd, LORA_SET_BANDWIDTH, &bw);
	printf("Going to set the RF bandwith %u Hz\n", bw);

	/* Set the RF power. */
	int32_t power = 10;
	set_power(fd, power);
	printf("Going to set the RF power %d dbm\n", power);

	printf("The current RSSI is %d dbm\n", get_rssi(fd));

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
	uint32_t frq;
	ioctl(fd, LORA_GET_FREQUENCY, &frq);
	printf("The LoRa carrier frequency is %u Hz\n", frq);

	printf("The RF spreading factor is %u chips\n", get_sprfactor(fd));

	/* Get the RF bandwidth. */
	bw = 0;
	ioctl(fd, LORA_GET_BANDWIDTH, &bw);
	printf("The RF bandwith is %u Hz\n", bw);

	printf("The current RSSI is %d dbm\n", get_rssi(fd));
	printf("The last packet SNR is %u db\n", get_snr(fd));
	printf("The output power is %d dbm\n", get_power(fd));

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
