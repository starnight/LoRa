cd LoRa
make clean; make; make install

cd ../LoRa-SPI
make clean; make; make install; echo

cat /proc/kallsyms | grep example; echo
ls -l /dev/useexample*

cd ../test-application
make; make test; make clean

cd ../LoRa-SPI
make uninstall; make clean

cd ../LoRa
make uninstall; make clean

dmesg | tail -n 50
