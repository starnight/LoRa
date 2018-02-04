PROJ=lrw
obj-m := $(PROJ).o
$(PROJ)-objs := lorawan.o lrwsec.o

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
	sudo rm $(KERNEL_LOCATION)/extra/$(PROJ).ko.[bgx]z
	# Rebuild the kernel module dependencies for modprobe
	sudo depmod -a

clean:
	make -C $(BUILDDIR) M=$(PWD) clean
