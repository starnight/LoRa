# Test Applications

These applications communicate through UDP/IPv6 socket in simple client-server model.

1. Client will send a data string to server.
2. Server will capitalize the recieved data string from the client and send back to the client.
3. Client will receive the capitalized data string and print it out.

## Setup a 6LoWPAN Test Network

Refer to: linux-wpan http://wpan.cakelab.org/

### wpan-tools

1. Could be founded at https://github.com/linux-wpan/wpan-tools
2. The dependencies will be listed during building

### Hava a lowpan interface from a wpan interface

**SX1278 driver** and **6LoWPAN kernel module** should be inserted or loaded before this action.

Do these works with the granted privilege.

```sh
# Private Area Network ID
panid="0xbeef"
# Index of the wpan interface
i=0

# Set the PANID of the wpan interface
iwpan dev wpan${i} set pan_id $panid
# Create a lowpan interface over the wpan interface
ip link add link wpan${i} name lowpan${i} type lowpan
# Bring up the wpan and lowpan interfaces
ip link set wpan${i} up
ip link set lowpan${i} up
```

```ip addr``` will show the IPv6 addresses of the interfaces.

## Test Applications

### Build test applications

```make``` will produce **server** and **client**.

### server

```server <listening IPv6 address> <listening port>```

- listening IPv6 address:
  Listening on which IPv6 address

- listening port:
  Listening on which UDP port

### client

```client <src IPv6 address> <dst IPv6 address> <dst port> <data string>```

- src IPv6 address:
  Send with the source IPv6 address

- dst IPv6 address:
  Send to the server's IPv6 address

- dst port:
  Send to the server's UDP port

- data string:
  Send the data string to the server
