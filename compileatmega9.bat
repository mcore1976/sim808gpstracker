del main9.elf
del main9.o
del main9.hex
avr-gcc -mmcu=atmega328p -std=gnu99 -Wall -Os -o main9.elf main9.c -w
avr-objcopy -j .text -j .data -O ihex main9.elf main9.hex
avr-size --mcu=atmega328p --format=avr main9.elf
# fuse = 62 for 1MHz clock = internal 8Meg / division 8
# ATTENTION ! if using External XTAL 8Hz with division by 8 - replace "lfuse:w:0x7f:m" below
avrdude -c usbasp -p m328p -U lfuse:w:0x62:m  -U flash:w:"main9.hex":a

