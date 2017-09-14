# LoRa
This is a LoRa device driver as a Linux kernel module with IEEE 802.15.4 MAC interfaces.

The driver with file operation interfaces could be found at the *[file-ops branch](https://github.com/starnight/LoRa/tree/file-ops)*.

## Compatible Chips
* Semtech SX1276/77/78/79

## Folders
* LoRa: The LoRa header source and build files.
* dts-overlay: The device tree overlayers with the boards and operating systems.
* test-application: The user space applications for testing or demo.

## Build and Install

1. Build
```sh
cd LoRa
make
```

2. Install
```sh
make install
```

3. Load module
```sh
modprobe sx1278
```
  If the target uses Device Tree mechanism like some embedded systems, Raspberry Pi for example.
  Its device tree may need to be updated first.
  There is a device tree overlay for Raspberry Pi in the dts-overlay folder for example.
  Just ``` make ``` in the folder, than it will compile and install the device tree overlay, and reboot is needed.

4. Check the installed module
```sh
dmesg
```

## License
Under Dual BSD/GPL
