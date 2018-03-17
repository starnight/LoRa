CC=cc

PROJ1=send
SRC1=$(PROJ1).c lora-ioctl.c
DEV1=/dev/lora0

PROJ2=receive
#SRC2=$(PROJ2).c lora-ioctl.c
#DEV2=/dev/loraSPI0.1

all:
	$(CC) $(SRC1) -o $(PROJ1)
#	$(CC) $(SRC2) -o $(PROJ2)

test:
	sudo ./$(PROJ1) $(DEV1) sdfsdfds
#	sudo ./$(PROJ2) $(DEV2)

clean:
	rm $(PROJ1) $(PROJ2)
