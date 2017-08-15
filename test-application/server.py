#!/usr/bin/env python3

import sys
import socket

MES_PORT = 8000

class Message:
    '''Message class.'''
    def __init__(self):
        self._Len = 0
        self._Buf = None

def fetch_ipv6_address(host, port):
    '''Get designated IPv6 and Port socket address.'''
    if not socket.has_ipv6:
        raise Exception("the local machine has no IPv6 support enabled")

    addrs = socket.getaddrinfo(host, port, socket.AF_INET6, 0, socket.SOL_TCP)
    if len(addrs) == 0:
        raise Exception("there is no IPv6 address configured for localhost")

    entry0 = addrs[0]
    sockaddr = entry0[-1]

    return sockaddr

if __name__ == "__main__":
    if len(sys.argv) > 2:
        srv_addr = sys.argv[1]
        srv_port = int(sys.argv[2])
    elif len(sys.argv) > 1:
        srv_addr = sys.argv[1]
        srv_port = MES_PORT
    else:
        srv_addr = "::"
        srv_port = MES_PORT

    req = Message()
    res = Message()

    # Connect to destination server with specific source IPv6.
    conn = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
    srv_sockaddr = fetch_ipv6_address(srv_addr, srv_port)
    conn.bind(srv_sockaddr)

    print("Server is started!!! Listening on {} UDP port {}".format(srv_addr, srv_port))
    while True:
        # Read and parse the response from the server.
        data, cli_addr = conn.recvfrom(1024)
        print("Client from {} port {}".format(cli_addr[0], cli_addr[1]))
        print(data)
        req._Len = int.from_bytes([data[0]], byteorder='big')
        req._Buf = data[1:req._Len + 1]#conn.recvfrom(req._Len)
        print("\tRead {} with in {} bytes from {}".format(req._Buf, req._Len, cli_addr[0]))

        # Prepare the request buffer going to be sent.
        res._Len = req._Len
        res._Buf = req._Buf.upper()

        # Send the request to the server.
        print("\tSend {} with in {} bytes".format(res._Buf, res._Len))
        conn.sendto(bytes([res._Len]) + res._Buf, cli_addr)

    conn.close()
