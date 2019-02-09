OVERLAY_SRC=rpi-lora-spi-overlay.dts
OVERLAY_BIN=rpi-lora-spi-overlay.dtbo
OVERLAY_DST=/boot/overlays/rpi-lora-spi.dtbo

all:
	dtc -I dts -O dtb -@ -o $(OVERLAY_BIN) $(OVERLAY_SRC)

install:
	sudo cp $(OVERLAY_BIN) $(OVERLAY_DST)

uninstall:
	sudo rm $(OVERLAY_DST)

test:
	dtc -I fs /proc/device-tree

clean:
	rm $(OVERLAY_BIN)
