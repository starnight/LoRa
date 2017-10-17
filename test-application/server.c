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
	hints.ai_socktype = SOCK_DGRAM;

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
	int yes = 1;

	addr = have_addr(ipv6, port);
	if (!addr)
		return -1;

	s = socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
	setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
	if (bind(s, addr->ai_addr, addr->ai_addrlen)) {
		perror("bind socket failed");
		return -1;
	}

	freeaddrinfo(addr);

	return s;
}

int main(int argc, char *argv[])
{
	int srvsock;
	struct sockaddr_in6 cli_addr;
	socklen_t addrlen;

#define	BUFLEN		(1024)
	char buf[BUFLEN];
	ssize_t buflen;
	int i;

	if (argc < 3) {
		printf("Usage: server <srv_addr> <srv_port>\n");
		return 0;
	}
	char *srv_ip = argv[1];
	char *srv_port = argv[2];

	/* Have the server socket. */
	srvsock = have_bound_socket(srv_ip, srv_port);
	if (srvsock < 0)
		return srvsock;

	printf("Server is started!!! Listening on %s UDP port %s\n",
	       srv_ip, srv_port);
	while (1) {
		/* Prepare and receive from client. */
		memset(buf, 0, BUFLEN);
		addrlen = sizeof(cli_addr);
		buflen = recvfrom(srvsock, buf, BUFLEN, 0,
				  (struct sockaddr *)&cli_addr, &addrlen);
		if (buflen < 0) {
			perror("receive from client failed");
			continue;
		}
		
		/* Show client's information. */
		printf("Client ");
		show_addr_info(&cli_addr);
		printf("\tRecv %s with in %zd bytes\n", buf, buflen);

		/* Uppercase received characters in buffer. */
		for (i = 0; buf[i] != 0; i++)
			buf[i] = toupper(buf[i]);

		/* Send the processed buffer back to client. */
		printf("\tSend %s with in %zu bytes\n", buf, strlen(buf));
		if (sendto(srvsock, buf, strlen(buf), 0,
			   (struct sockaddr *)&cli_addr, addrlen) < 0) {
			perror("send to client failed");
		}
	}

	close(srvsock);

	return 0;
}
