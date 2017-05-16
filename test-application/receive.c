#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>

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
	int fd;
	char pstr[40];
#define MAX_BUFFER_LEN	16
	char buf[MAX_BUFFER_LEN];
	int len;

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

	/* Read from the file descriptor if it is ready to be read. */
	memset(buf, 0, MAX_BUFFER_LEN);
	printf("Going to read %s\n", path);
	len = do_read(fd, buf, MAX_BUFFER_LEN);
	printf("Read %s\n", path);

	/* Echo */
	printf("Going to echo %s\n", path);
	do_write(fd, buf, len);
	printf("Echoed %s\n", path);

	/* Close device node. */
	close(fd);

	return 0;
}
