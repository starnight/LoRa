#!/usr/bin/env python3

import sys
import socket
import select
import datetime

MES_PORT = 8000
MAX_ECHO_CLIENT = 5
ECHO_SERVER = "Micro PyECHO Server"

class ECHOError(Exception):
    '''Define an ECHO error exception.'''
    def __init__(self, message, error=1):
        super(ECHOError, self).__init__(message)
        self.error = error

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

def _ECHOWork(conn, req, res):
    '''Default Hello page which makes response message.'''
    res._Buf = bytes([req._Len]) + req._Buf.upper()

class ECHOServer:
    global MES_PORT
    global MAX_ECHO_CLIENT

    def __init__(self, host="::", port=MES_PORT):
        self.HOST = host
        self.PORT = port
        self.MAX_CLIENT = MAX_ECHO_CLIENT
        self._insocks = []
        self._outsocks = []
        # Create server socket.
        self.sock = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
        # Set the socket could be reused directly after this application be closed.
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Set the socket non-blocking.
        sockaddr = fetch_ipv6_address(self.HOST, self.PORT)
        self.sock.bind(sockaddr)

        # Start server socket listening.
        self.sock.listen(self.MAX_CLIENT)
        self._insocks.append(self.sock)

    def Run(self, callback):
        readable, writeable, exceptional = select.select(self._insocks, self._outsocks, self._insocks)
        for s in readable:
            if s is self.sock:
                self._Accept(s)
            else:
                self._Request(s, callback)

        for s in writeable:
            s.send("")
            self._outsocks.remove(s)
            s.close()

        for s in exceptional:
            self._insocks.remove(s)
            if s in self._outsocks:
                self._outsocks.remove(s)
            s.close()

    def RunLoop(self, callback):
        while True:
            self.Run(callback)

    def _Accept(self, conn):
        conn, addr = self.sock.accept()
        print("{} {} connected".format(str(datetime.datetime.now()), addr))
        self._insocks.append(conn)

    def _Request(self, conn, callback):
        request = Message()
        response = Message()
        try:
            self._GetRequest(conn, request)
            callback(conn, request, response)
            self._SendReply(conn, response)
        except ECHOError as e:
            print("\t{}".format(e))
        except Exception as e:
            print("\t{}".format(e))
        finally:
            self._insocks.remove(conn)
            if conn in self._outsocks:
                self._outsocks.remove(conn)

            conn.close()
        del request
        del response

    def _GetRequest(self, conn, req):
        '''Read the request message.'''
        req._Len = int.from_bytes(conn.recv(1), byteorder='big')
        print("\tRead request with length {}".format(req._Len))
        req._Buf = conn.recv(req._Len)

    def _SendReply(self, conn, res):
        '''Send the response message.'''
        print("\tSend reply")
        if res._Buf is not None:
            buf = res._Buf
            size = len(buf)
            totalsent = 0
            while totalsent < size:
                sent = conn.send(buf[totalsent:])
                if sent == 0:
                    raise ECHOError("Send connection broken")
                totalsent += sent

    def __del__(self):
        print("Close socket")
        self.sock.close()
        for s in self._insocks:
            s.close()
        for s in self._outsocks:
            s.close()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        ip_addr = sys.argv[1]
    else:
        ip_addr = "::"

    port=MES_PORT

    print("Server is starting!!!")
    server = ECHOServer(ip_addr, port)
    print("Server is started!!!  Listening on {} port {}".format(ip_addr, port))
    server.RunLoop(_ECHOWork)
