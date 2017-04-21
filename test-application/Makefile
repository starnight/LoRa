PROJ=test-application

all:
	cc main.c -o $(PROJ)

test:
	sudo ./$(PROJ) /dev/loraSPI0.0
	sudo ./$(PROJ) /dev/loraSPI0.1

clean:
	rm $(PROJ)
