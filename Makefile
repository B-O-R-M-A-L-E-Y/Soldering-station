TARG=main

CC = avr-gcc
OBJCOPY = avr-objcopy

# Задаем из каких файлов собирать проект, можно указать несколько файлов
SRCS= main.c pid.c encoder.c

OBJS = $(SRCS:.c=.o)

# Задаем для какого микроконтроллера будем компилировать (atmega8) -DF_CPU=11059200
MCU=atmega328p

# Флаги компилятора, при помощи F_CPU определяем частоту на которой будет работать контроллер,
CFLAGS = -mmcu=$(MCU) -std=c11 -Wall -g -Os -Werror -lm  -mcall-prologues
LDFLAGS = -mmcu=$(MCU)  -Wall -g -Os  -Werror

all: $(TARG)

$(TARG): $(OBJS)
	$(CC) $(LDFLAGS) -o $@.elf  $(OBJS) -lm
	$(OBJCOPY) -O binary -R .eeprom -R .nwram  $@.elf $@.bin
	$(OBJCOPY) -O ihex -R .eeprom -R .nwram  $@.elf $@.hex

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

flash:
	avrdude -b 19200 -c avrisp -p m328p -P /dev/ttyUSB0 -U flash:w:main.hex

clean:
	rm -f *.elf *.bin *.hex  $(OBJS) *.map
