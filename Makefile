TARGET = usb_device_hid

# Adjust the XRAM location and size to leave space for the USB DMA buffers
# Buffer layout in XRAM:
#__xdata __at (0x0000) uint8_t  Ep0Buffer[64];
#__xdata __at (0x0040) uint8_t  Ep4Buffer[16];
#__xdata __at (0x0050) uint8_t  Ep1Buffer[16];
#__xdata __at (0x0060) uint8_t  Ep2Buffer[16];
#__xdata __at (0x0070) uint8_t  Ep3Buffer[16];
#
# 128 bytes are reserved for USB endpoints, leaving 384 bytes available on CH551.
XRAM_SIZE = 0x00C0
XRAM_LOC = 0x0140

C_FILES = \
	main.c \
	fournsnes.c \
	include/debug.c

pre-flash:


include Makefile.include
