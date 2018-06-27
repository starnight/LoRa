CC=cc

PROJ1=send
SRC1=$(PROJ1).c lora-ioctl.c
DEV1=/dev/lora0

all:
	$(CC) $(SRC1) -o $(PROJ1)

test:
	sudo ./$(PROJ1) $(DEV1) sdfsdfds

clean:
	rm $(PROJ1) $(PROJ2)
