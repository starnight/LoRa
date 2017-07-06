PPROJ=lora
PROJ=lora-spi
obj-m := $(PROJ).o
$(PROJ)-objs := lora.o lora_spi.o sx1278.o

KERNEL_LOCATION=/lib/modules/$(shell uname -r)
BUILDDIR=$(KERNEL_LOCATION)/build

all:
	make -C $(BUILDDIR) M=$(PWD) modules

install:
	sudo make -C $(BUILDDIR) M=$(PWD) modules_install
	# Rebuild the kernel module dependencies for modprobe
	sudo depmod -a

uninstall:
	sudo modprobe -r $(PROJ)
	sudo rm $(KERNEL_LOCATION)/extra/$(PROJ).ko.gz
	# Rebuild the kernel module dependencies for modprobe
	sudo depmod -a

test:
	make install; echo
	ls -l /dev/$(PROJ)*
	make uninstall

clean:
	make -C $(BUILDDIR) M=$(PWD) clean
