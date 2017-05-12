PROJ1=send
SRC1=$(PROJ1).c
DEV1=/dev/loraSPI0.0

PROJ2=receive
SRC2=$(PROJ2).c
DEV2=/dev/loraSPI0.1

all:
	cc $(SRC1) -o $(PROJ1)
	cc $(SRC2) -o $(PROJ2)

test:
	sudo ./$(PROJ1) $(DEV1)
	sudo ./$(PROJ2) $(DEV2)

clean:
	rm $(PROJ1) $(PROJ2)
