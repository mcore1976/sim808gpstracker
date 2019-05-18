del main8.elf
del main8.o
del main8.hex
avr-gcc -mmcu=atmega328p -std=gnu99 -Wall -Os -o main8.elf main8.c -w
avr-objcopy -j .text -j .data -O ihex main8.elf main8.hex
avr-size --mcu=atmega328p --format=avr main8.elf
# fuse = 62 for 1MHz clock = internal 8Meg / division 8
avrdude -c usbasp -p m328p -U lfuse:w:0x62:m  -U flash:w:"main8.hex":a

