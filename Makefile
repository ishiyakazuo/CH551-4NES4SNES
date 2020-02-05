TARGET = 4nes4snes

# Adjust the XRAM location and size to leave space for the USB DMA buffers
XRAM_SIZE = 0x00C0
XRAM_LOC = 0x0140

C_FILES = \
	main.c \
	fournsnes.c \
	include/debug.c

pre-flash:


include Makefile.include
