del main2.elf
del main2.o
del main2.hex
avr-gcc -mmcu=atmega328p -std=gnu99 -Wall -Os -o main2.elf main2.c -w
avr-objcopy -j .text -j .data -O ihex main2.elf main2.hex
avr-size --mcu=atmega328p --format=avr main2.elf
# fuse = 62 for internal 8Meg with div 8 = 1MHz
avrdude -c usbasp -p m328p -U lfuse:w:0x62:m  -U flash:w:"main2.hex":a

