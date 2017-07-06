cd LoRa
make clean; make; make install

cd ../test-application
make; make test; make clean

cd ../LoRa
make uninstall; make clean

dmesg | tail -n 50
