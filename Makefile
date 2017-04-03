PROJ=lora
obj-m := $(PROJ).o

KERNEL_LOCATION=/lib/modules/$(shell uname -r)
BUILDDIR=$(KERNEL_LOCATION)/build

OVERLAY_SRC=rpi-lora-spi-overlay.dts
OVERLAY_DST=/boot/overlays/rpi-lora-spi.dtbo

all:
	make -C $(BUILDDIR) M=$(shell pwd) modules

dts:
	sudo dtc -I dts -O dtb -@ -o $(OVERLAY_DST) $(OVERLAY_SRC)

install:
	gzip $(PROJ).ko
	sudo mv $(PROJ).ko.gz $(KERNEL_LOCATION)/kernel/drivers/spi/lora-spi.ko.gz
	# Rebuild the kernel module dependencies for modprobe
	sudo depmod -a

test-pre:
	sudo insmod ./$(PROJ).ko
	dmesg | tail
	cat /sys/class/$(PROJ)/$(PROJ)/dev
	cat /sys/class/$(PROJ)/$(PROJ)/uevent
	sudo chmod 666 /dev/$(PROJ)
	ls -l /dev/$(PROJ)
	#cc test-application/main.c -o ioctl

test-action:
	#cat /dev/$(PROJ)
	#dmesg | tail -n 40
	#echo Happy! > /dev/$(PROJ)
	#dmesg | tail -n 40
	#./ioctl /dev/$(PROJ) GET
	#./ioctl /dev/$(PROJ) SET 2
	#./ioctl /dev/$(PROJ) GET
	#dmesg | tail
	#cat /dev/$(PROJ)
	#dmesg | tail -n 40
	#echo GoGoGoGoGoGo! > /dev/$(PROJ)
	#dmesg | tail -n 40
	#cat /dev/$(PROJ)
	#dmesg | tail -n 40

test-post:
	sudo rmmod $(PROJ)
	dmesg | tail

test: test-pre test-action test-post

clean:
	make -C $(BUILDDIR) M=$(shell pwd) clean
	rm -f ioctl
