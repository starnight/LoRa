OVERLAY_SRC=rpi-lora-spi-overlay.dts
OVERLAY_DST=/boot/overlays/rpi-lora-spi.dtbo

all:
	sudo dtc -I dts -O dtb -@ -o $(OVERLAY_DST) $(OVERLAY_SRC)

test:
	dtc -I fs /proc/device-tree
