PROJ=fakelr
obj-m := $(PROJ).o

KERNEL_LOCATION=/lib/modules/$(shell uname -r)
BUILDDIR=$(KERNEL_LOCATION)/build

all:
	make -C $(BUILDDIR) M=$(PWD) modules

install:
	sudo make -C $(BUILDDIR) M=$(PWD) modules_install
	# Rebuild the kernel module dependencies for modprobe
	sudo depmod -a
	sudo modprobe $(PROJ)

uninstall:
	sudo modprobe -r $(PROJ)
	sudo rm $(KERNEL_LOCATION)/extra/$(PROJ).ko.[bgx]z
	sudo depmod -a

test:
	make install; echo
	ls -l /dev/lora*
	make uninstall

clean:
	make -C $(BUILDDIR) M=$(PWD) clean
