#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

void show_addr_info(struct sockaddr_in6 *addr)
{
	char ipv6[INET6_ADDRSTRLEN];
	int port;

	inet_ntop(AF_INET6, &(addr->sin6_addr), ipv6, INET6_ADDRSTRLEN);
	port = ntohs(addr->sin6_port);

	printf("address %s port %d\n", ipv6, port);
}

struct addrinfo * have_addr(char *ipv6, char *port)
{
	struct addrinfo hints;
        struct addrinfo *addr;
	int status;

	memset(&hints, 0, sizeof(struct addrinfo));
	hints.ai_family = PF_INET6;
	hints.ai_socktype = SOCK_STREAM;

	status = getaddrinfo(ipv6, port, &hints, &addr);
	if (status) {
		perror("getaddrinfo failed");
		return NULL;
	}
	if (!addr) {
		fprintf(stderr, "no interface with %s\n", ipv6);
		return NULL;
	}

	return addr;
}

int have_bound_socket(char *ipv6, char *port)
{
	int s;
	struct addrinfo *addr;

	addr = have_addr(ipv6, port);
	if (!addr)
		return -1;

	s = socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
	if (bind(s, addr->ai_addr, addr->ai_addrlen)) {
		perror("bind socket failed");
		return -1;
	}

	freeaddrinfo(addr);

	return s;
}

int main(int argc, char *argv[])
{
	int conn;
	struct addrinfo *dst_addr;

#define	BUFLEN		(1024)
	char buf[BUFLEN];
	size_t buflen;

	char *src_ip = argv[1];
	char *dst_ip = argv[2];
	char *dst_port = argv[3];
	char *data_str = argv[4];

	/* Have the server socket. */
	conn = have_bound_socket(src_ip, NULL);
	if (conn < 0)
		return conn;

	/* Have server's address information structure and connect to it. */
	dst_addr = have_addr(dst_ip, dst_port);
	if (connect(conn, dst_addr->ai_addr, dst_addr->ai_addrlen) < 0) {
		perror("connect to server failed");
		freeaddrinfo(dst_addr);
		return -1;
	}
	freeaddrinfo(dst_addr);

	/* Send the data string to server. */
	printf("Send %s with in %d bytes\n", data_str, strlen(data_str));
	if (send(conn, data_str, strlen(data_str), 0) < 0) {
		perror("send to server failed");
		return -1;
	}

	/* Prepare and receive from server. */
	memset(buf, 0, BUFLEN);
	buflen = recv(conn, buf, BUFLEN, 0);
	if (buflen < 0) {
		perror("receive from server failed");
		return -1;
	}
	printf("Recv %s with in %zd bytes\n", buf, buflen);

	close(conn);

	return 0;
}
