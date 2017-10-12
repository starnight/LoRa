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
	int srvsock, clisock;
	struct sockaddr_in6 cli_addr;
	socklen_t addrlen;

#define	BUFLEN		(1024)
	char buf[BUFLEN];
	size_t buflen;
	int i;

	char *srv_ip = argv[1];
	char *srv_port = argv[2];

	/* Have the server socket. */
	srvsock = have_bound_socket(srv_ip, srv_port);
	if (srvsock < 0)
		return srvsock;

	/* Listening. */
	listen(srvsock, 1);
	printf("Server is started!!! Listening on %s TCP port %s\n",
	       srv_ip, srv_port);

	while (1) {
		/* Accept a client. */
		addrlen = sizeof(cli_addr);
		clisock = accept(srvsock,
				 (struct sockaddr *)&cli_addr,
				 &addrlen);
		if (clisock < 0)
			continue;

		/* Show client's information. */
		printf("Client ");
		show_addr_info(&cli_addr);

		/* Prepare and receive from client. */
		memset(buf, 0, BUFLEN);
		buflen = recv(clisock, buf, BUFLEN, 0);
		if (buflen <= 0) {
			perror("receive from client failed");
			continue;
		}
		printf("\tRecv %s with in %zd bytes\n", buf, buflen);

		/* Uppercase received characters in buffer. */
		for (i = 0; buf[i] != 0; i++)
			buf[i] = toupper(buf[i]);

		/* Send the processed buffer back to client. */
		printf("\tSend %s with in %d bytes\n", buf, strlen(buf));
		if (send(clisock, buf, strlen(buf), 0) < 0)
			perror("send to client failed");

		close(clisock);
	}

	close(srvsock);

	return 0;
}
