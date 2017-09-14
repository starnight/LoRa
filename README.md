# LoRa
This is a LoRa device driver as a Linux kernel module with file operation interfaces.

The driver with IEEE 802.15.4 MAC interfaces could be found at the *[master branch](https://github.com/starnight/LoRa)*.

## Compatible Chips
* Semtech SX1276/77/78/79

## Folders
* LoRa: The general LoRa header and source files and the LoRa over SPI device driver.
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

## Usage
After build and install successfully, there is ```/dev/loraSPIX.Y``` corresponding to the LoRa chip which could be _open_, _read_, _write_, _ioctl_, _select_, _close_ ... directly.

## License
Under Dual BSD/GPL
