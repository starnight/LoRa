#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
//#include <fcntl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/sockios.h>

//#include "lora-ioctl.h"

#define	PF_LORAWAN	PF_MAX
#define	AF_LORAWAN	PF_LORAWAN

enum {
	LRW_ADDR_APPEUI,
	LRW_ADDR_DEVEUI,
	LRW_ADDR_DEVADDR,
};

struct lrw_addr_in {
	int addr_type;
	union {
		uint64_t app_eui;
		uint64_t dev_eui;
		uint32_t devaddr;
	};
};

struct sockaddr_lorawan {
	sa_family_t family; /* AF_LORAWAN */
	struct lrw_addr_in addr_in;
};

/* DEV ioctl() commands */

enum LRW_IOC_CMDS {
        SIOCGLRWSTATE = SIOCDEVPRIVATE,
        SIOCSLRWSTATE,
        SIOCGLRWFREQ,
        SIOCSLRWFREQ,
        SIOCGLRWBW,
        SIOCSLRWBW,
        SIOCGLRWTXPWR,
        SIOCSLRWTXPWR,
        SIOCGLRWSPRF,
        SIOCSLRWSPRF,
        SIOCGLRWLNA,
        SIOCSLRWLNA,
        SIOCGLRWRSSI,
        SIOCGLRWSNR,
};

/* Bit 1: 1 for ready to read, 0 for not ready
 * Bit 0: 1 for ready to write, 0 for not write
 */
//uint8_t ready2rw(int fd)
//{
//	fd_set read_fds, write_fds;
//	struct timeval tv = {.tv_sec = 5, .tv_usec = 0};
//	uint8_t flag;
//
//	/* I/O multiplexing. */
//	FD_ZERO(&read_fds);
//	FD_ZERO(&write_fds);
//	FD_SET(fd, &read_fds);
//	FD_SET(fd, &write_fds);
//	if (select(fd+1, &read_fds, &write_fds, NULL, &tv) == -1)
//		perror("Select failed");
//
//	flag = 0;
//	/* Read from the file descriptor if it is ready to be read. */
//	if (FD_ISSET(fd, &read_fds)) {
//		flag |= (1 << 1);
//	}
//	/* Write to the file descriptor if it is ready to be written. */
//	if (FD_ISSET(fd, &write_fds)) {
//		flag |= (1 << 0);
//	}
//
//	return flag;
//}

//#define ready2read(fd)	(ready2rw(fd) & (1 << 1))
//#define ready2write(fd)	(ready2rw(fd) & (1 << 0))

int main(int argc, char **argv)
{
	char *data;
	char pstr[40];
	int sock;
	struct sockaddr_lorawan src_addr, dst_addr;
	struct ifreq ifr;
	int32_t pwr = 100;
#define MAX_BUFFER_LEN	16
	char buf[MAX_BUFFER_LEN];
	int len;
	unsigned int s;
	int ret;

	/* Parse command. */
	if (argc >= 2) {
		//path = argv[1];
		data = argv[1];
	}
	else {
		printf("Need more arguments.\r\n");
		return -1;
	}

	sock = socket(PF_LORAWAN, SOCK_DGRAM, 0);
	if (sock < 0) {
		perror("socket error\n");
		return -1;
	}

	src_addr.family = AF_LORAWAN;
	src_addr.addr_in.addr_type = LRW_ADDR_DEVADDR;
	src_addr.addr_in.devaddr = 0x01020304;
	printf("Going to bind address %X\n", src_addr.addr_in.devaddr);
	ret = bind(sock, (struct sockaddr *)&src_addr, sizeof(src_addr));
	if (sock < 0) {
		perror("bind error\n");
		return ret;
	}

	strcpy(ifr.ifr_name, "lora0");
	ifr.ifr_data = (void *)&pwr;
	ret = ioctl(sock, SIOCSLRWTXPWR, &ifr);
	if(ioctl(sock, SIOCSLRWTXPWR, &ifr) < 0) {
		perror("ioctl error");
		return ret;
	}

	dst_addr.family = AF_LORAWAN;
	dst_addr.addr_in.addr_type = LRW_ADDR_DEVADDR;
	dst_addr.addr_in.devaddr = 0xFFFFFFFF;
	len = sendto(sock, data, strlen(data), 0, (struct sockaddr *)&dst_addr, sizeof(dst_addr));
	if (len < 0) {
		perror("sendto");
	}

//	/* Open device node. */
//	fd = open(path, O_RDWR);
//	if (fd == -1) {
//		sprintf(pstr, "Open %s failed", path);
//		perror(pstr);
//		return -1;
//	}
//	printf("Opened %s\n", path);
//
//	set_state(fd, LORA_START);
//	/*Set the RF spreading factor. */
//	uint32_t sprf = 2048;
//	set_sprfactor(fd, sprf);
//	printf("Going to set the RF spreading factor %u chips\n", sprf);
//
//	/* Set the RF bandwidth. */
//	uint32_t bw = 125000;
//	set_bw(fd, bw);
//	printf("Going to set the RF bandwith %u Hz\n", bw);
//
//	/* Set the RF power. */
//	int32_t power = 10;
//	set_power(fd, power);
//	printf("Going to set the RF power %d dbm\n", power);
//
//	printf("The current RSSI is %d dbm\n", get_rssi(fd));
//
//	/* Write to the file descriptor if it is ready to be written. */
//	printf("Going to write %s\n", path);
//	s = 0;
//	while (!ready2write(fd)) {
//		sleep(1);
//		s++;
//		printf("\t%s is not ready to write, do other things.", path);
//		printf("  %u s\r", s);
//	}
//	printf("\n");
//	len = do_write(fd, data, strlen(data));
//	if (len < 0)
//		perror("Error");
//	else
//		printf("Written %d bytes: %s\n", len, data);
//
//	/* Read from echo if it is ready to be read. */
//	memset(buf, 0, MAX_BUFFER_LEN);
//	printf("Going to read %s\n", path);
//	/* Set the device in read state. */
//	set_state(fd, LORA_STATE_RX);
//	s = 0;
//	while (!ready2read(fd)) {
//		sleep(1);
//		s++;
//		printf("\t%s is not ready to read, do other things.", path);
//		printf("  %u s\r", s);
//	}
//	printf("\n");
//	len = do_read(fd, buf, MAX_BUFFER_LEN - 1);
//	if (len > 0)
//		printf("Read %d bytes: %s\n", len, buf);
//
//	printf("The LoRa carrier frequency is %u Hz\n", get_freq(fd));
//	printf("The RF spreading factor is %u chips\n", get_sprfactor(fd));
//	printf("The RF bandwith is %u Hz\n", get_bw(fd));
//	printf("The current RSSI is %d dbm\n", get_rssi(fd));
//	printf("The last packet SNR is %d db\n", get_snr(fd));
//	printf("The output power is %d dbm\n", get_power(fd));
//	printf("The LNA gain is %d db\n", get_lna(fd));
//
//	/* Set the device in sleep state. */
//	set_state(fd, LORA_STATE_SLEEP);
//	printf("The LoRa device is in 0x%X state\n", get_state(fd));

	sleep(10);
//	set_state(fd, LORA_STOP);
	/* Close device node. */
	shutdown(sock, SHUT_RDWR);
	close(sock);

	return 0;
}
