#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>

#include "lora-ioctl.h"

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

	/*Set the RF spreading factor. */
	uint32_t sprf = 2048;
	set_sprfactor(fd, sprf);
	printf("Going to set the RF spreading factor %u chips\n", sprf);

	/* Set the RF bandwidth. */
	uint32_t bw = 125000;
	set_bw(fd, bw);
	printf("Going to set the RF bandwith %u Hz\n", bw);

	/* Set the RF power. */
	int32_t power = 10;
	set_power(fd, power);
	printf("Going to set the RF power %d dbm\n", power);

	printf("The current RSSI is %d dbm\n", get_rssi(fd));

	/* Read from the file descriptor if it is ready to be read. */
	memset(buf, 0, MAX_BUFFER_LEN);
	printf("Going to read %s\n", path);
	len = do_read(fd, buf, MAX_BUFFER_LEN - 1);

	if(len > 0) {
		printf("Read %d bytes: %s\n", len, buf);
		printf("The current RSSI is %d dbm\n", get_rssi(fd));
		printf("The last packet SNR is %u db\n", get_snr(fd));

		sleep(1);

		/* Echo */
		printf("Going to echo %s\n", path);
		for(i = 0; i < len; i++)
			buf[i] = toupper(buf[i]);
		len = do_write(fd, buf, len);
		printf("Echoed %d bytes: %s\n", len, buf);
	}

	printf("The LoRa carrier frequency is %u Hz\n", get_freq(fd));
	printf("The RF spreading factor is %u chips\n", get_sprfactor(fd));
	printf("The RF bandwith is %u Hz\n", get_bw(fd));
	printf("The output power is %d dbm\n", get_power(fd));

	/* Set the device in sleep state. */
	set_state(fd, LORA_STATE_SLEEP);
	printf("The LoRa device is in 0x%X state\n", get_state(fd));

	/* Close device node. */
	close(fd);

	return 0;
}
