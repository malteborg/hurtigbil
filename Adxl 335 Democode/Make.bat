main:
	avr-gcc -mmcu=atmega32a -o main.bin *.c
	avr-objcopy -j .text -j .data -O ihex main.bin main.hex
	avrdude -p m32 -c usbasp -P usb -U flash:w:main.hex:i