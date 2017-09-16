# Test Applications

These applications communicate through UDP/IPv6 socket in simple client-server model.

1. Client will send a data string to server.
2. Server will capitalize the recieved data string from the client and send back to the client.
3. Client will receive the capitalized data string and print it out.

## server.py

```server.py <listening IPv6 address> <listening port>```

- listening IPv6 address:
  Listening on which IPv6 address

- listening port:
  Listening on which UDP port

## client.py

```lient.py <src IPv6 address> <dest IPv6 address> <dest port> <data string>```

- src IPv6 address:
  Send with the source IPv6 address

- dest IPv6 address:
  Send to the server's IPv6 address

- dest port:
  Send to the server's UDP port

- data string:
  Send the data string to the server
