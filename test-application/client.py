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
    if len(sys.argv) > 4:
        src_addr = sys.argv[1]
        dst_addr = sys.argv[2]
        dst_port = int(sys.argv[3])
        data = sys.argv[4]
    elif len(sys.argv) > 3:
        dst_addr = sys.argv[1]
        dst_port = int(sys.argv[2])
        data = sys.argv[3]
    else:
        src_addr = "::1"
        dst_addr = "::1"
        dst_port=MES_PORT
        data = "test string"

    req = Message()
    res = Message()

    # Connect to destination server with specific source IPv6.
    conn = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
    if 'src_addr' in locals():
        src_sockaddr = fetch_ipv6_address(src_addr, 0)
        conn.bind(src_sockaddr)
    dst_sockaddr = fetch_ipv6_address(dst_addr, dst_port)

    # Prepare the request buffer going to be sent.
    req._Len = len(data)
    if req._Len > 255:
        req._Len = 255
    req._Buf = str.encode(data)[0:req._Len]

    # Send the request to the server.
    print("Send {} with in {} bytes".format(req._Buf, req._Len))
    conn.sendto(bytes([req._Len]) + req._Buf, dst_sockaddr)

    # Read and parse the response from the server.
    data, addr = conn.recvfrom(1024)
    res._Len = int.from_bytes([data[0]], byteorder='big')
    res._Buf = data[1:res._Len + 1]#conn.recvfrom(res._Len)
    print("Read {} with in {} bytes".format(res._Buf, res._Len))

    conn.close()
