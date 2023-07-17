#Author: sboldenko
#Data  : 09.08.2022
#E-mail: venera.electronica@gmail.com

MCU=attiny13
F_CPU=9600000
CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-mmcu=${MCU} -DF_CPU=${F_CPU} -Os
TARGET=main
SRC=src/main.c

#AVRDUDE
PROGRAMMER=usbasp
AVRDUDE_MCU=t13

#FUSE BITS
LFUSE=0x7a
HFUSE=0xff

OUTDIR=build

all:
	mkdir -p $(OUTDIR)
	${CC} ${CFLAGS} -o $(OUTDIR)/${TARGET}.o ${SRC}
	${OBJCOPY} -j .text -j .data -O ihex $(OUTDIR)/${TARGET}.o $(OUTDIR)/${TARGET}.hex

signature:
	avrdude -c ${PROGRAMMER} -P usb  -p${AVRDUDE_MCU}

fuse:
	avrdude -c ${PROGRAMMER} -P usb -p ${AVRDUDE_MCU} -U lfuse:w:${LFUSE}:m -U hfuse:w:${HFUSE}:m

flash:
	avrdude -c ${PROGRAMMER} -P usb -p ${AVRDUDE_MCU} -U flash:w:$(OUTDIR)/${TARGET}.hex

clean:
	rm -R $(OUTDIR)