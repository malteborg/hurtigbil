main:
	avr-gcc -mmcu=atmega32 -o main.bin *.c
	avr-objcopy -j .text -j .data -O ihex main.bin main.hex
	avrdude -p m32 -c usbasp -P usb -U flash:w:main.hex:i

fuse8Mhz_External:
	avrdude -c usbasp -p m32 -U lfuse:w:0xff:m -U hfuse:w:0xd9:m

fuse8Mhz_Internal:
	avrdude -c usbasp -p m32 -U lfuse:w:0xe4:m -U hfuse:w:0xd9:m

fuse 4Mhz_Internal:
	avrdude -c usbasp -p m32 -U lfuse:w:0xe3:m -U hfuse:w:0xd9:m

fuse 2Mhz_Internal:
	avrdude -c usbasp -p m32 -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m

fuse 1Mhz_Internal:
	avrdude -c usbasp -p m32 -U lfuse:w:0xe1:m -U hfuse:w:0xd9:m
