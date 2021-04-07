/* Name: fournsnes.c
 * Project: Multiple NES/SNES to USB converter
 * Author: Raphael Assenat <raph@raphnet.net>
 * Copyright: (C) 2007-2016 Raphael Assenat <raph@raphnet.net>
 * License: GPLv2
 * Tabsize: 4
 */
#include "fournsnes.h"
#include "ch554.h"
#include "debug.h"
#include <stdio.h>
#include <string.h>

#define GAMEPAD_BYTES	8	/* 2 byte per snes controller * 4 controllers */

// SNES pinout: +5V, clock, latch, data, D1 (unused), I/O (unused), GND

/******** IO port definitions **************/
#define PORT1REG 0x90
#define PORT2REG 0xA0
#define PORT3REG 0xB0

#define LED_PIN1 0 // P3.0

#define MULTITAP_PIN 1 // P3.1
SBIT(MTAP_SIG, PORT3REG, MULTITAP_PIN);
#define LATCH_PIN 2 // P3.2
SBIT(LATCH_SIG, PORT3REG, LATCH_PIN);
#define CLOCK_PIN 3 // P3.3
SBIT(CLOCK_SIG, PORT3REG, CLOCK_PIN);
#define DATA0_PIN 4 // P1.4
SBIT(DATA0_SIG, PORT1REG, DATA0_PIN);
#define DATA1_PIN 5 // P1.5
SBIT(DATA1_SIG, PORT1REG, DATA1_PIN);
#define DATA2_PIN 6 // P1.6
SBIT(DATA2_SIG, PORT1REG, DATA2_PIN);
#define DATA3_PIN 7 // P1.7
SBIT(DATA3_SIG, PORT1REG, DATA3_PIN);

#define _delay_us mDelayuS

/********* IO port manipulation macros **********/
#define SNES_LATCH_LOW()	LATCH_SIG = 0
#define SNES_LATCH_HIGH()	LATCH_SIG = 1
#define SNES_CLOCK_LOW()	CLOCK_SIG = 0
#define SNES_CLOCK_HIGH()	CLOCK_SIG = 1

#define SNES_GET_DATA1()	DATA0_SIG
#define SNES_GET_DATA2()	DATA1_SIG
#define SNES_GET_DATA3()	DATA2_SIG
#define SNES_GET_DATA4()	DATA3_SIG

#define MTAP_SELECT_HIGH() MTAP_SIG = 1
#define MTAP_SELECT_LOW()	 MTAP_SIG = 0

/*********** prototypes *************/
// the most recent bytes we fetched from the controller
static __xdata uint8_t last_read_controller_bytes[GAMEPAD_BYTES];

static uint16_t incrementer = 0;

typedef struct controllerFlags {
    unsigned nesMode         : 4;
    unsigned fourscore_mode  : 1;
    unsigned multitap_mode   : 1;
    unsigned live_autodetect : 1;
} controllerFlags;

static struct controllerFlags ctrlFlags;
#if CONTROLLER_TYPE_SNES
void disableLiveAutodetect(void)
{
    ctrlFlags.live_autodetect = 0;
}

void enableLiveAutodetect(void)
{
    ctrlFlags.live_autodetect = 1;
}
#endif
static void autoDetectSNESMultiTap(void)
{
	// Detection is done by observing that DATA2 becomes
	// low when LATCH is high.
	//
	// Not sure which state of MTAP_SELECT_HIGH is reliable
	// so I'm simply trying with both states.

  ctrlFlags.multitap_mode = 0;
	MTAP_SELECT_LOW();

	if (SNES_GET_DATA2()) {
		SNES_LATCH_HIGH();
		_delay_us(12);

		if (!SNES_GET_DATA2()) {
			SNES_LATCH_LOW();
			_delay_us(12);
			if (SNES_GET_DATA2()) {
				ctrlFlags.multitap_mode = 1;
			}
		}
	}

	MTAP_SELECT_HIGH();

	if (SNES_GET_DATA2()) {
		SNES_LATCH_HIGH();
		_delay_us(12);

		if (!SNES_GET_DATA2()) {
			SNES_LATCH_LOW();
			_delay_us(12);
			if (SNES_GET_DATA2()) {
				ctrlFlags.multitap_mode = 1;
			}
		}
	}
}

static void autoDetectFourScore(void)
{
	unsigned char dat18th_low = 0;
	unsigned char hc=0;
	int i;

	SNES_LATCH_HIGH();
	_delay_us(12);
	SNES_LATCH_LOW();

	for (i=0; i<24; i++)
	{
		_delay_us(6);
		SNES_CLOCK_LOW();

		if (!SNES_GET_DATA1()) {
			if (i==19) {
				dat18th_low = 1;
			}
		}
		else {
			hc++;
		}

		_delay_us(6);
		SNES_CLOCK_HIGH();
	}

  ctrlFlags.fourscore_mode = 0;
	if (dat18th_low && hc == 23) {
		// only 18th data bit was low. Looks like a FOUR SCORE.
		ctrlFlags.fourscore_mode = 1;
		ctrlFlags.nesMode = 0x0F; // Set all 4 controllers to NES mode
	}
}

void fournsnesInit(void)
{
	// It just so happens that all of our port 3 pins are push/pull outputs, and all of our port 1 pins are inputs w/ pull-ups...
    P3_MOD_OC = P3_MOD_OC & ~((1<<LED_PIN1) | (1<<MULTITAP_PIN) | (1<<LATCH_PIN) | (1<<CLOCK_PIN)); // 0 = push/pull, 1 = open-drain
    P3_DIR_PU = P3_DIR_PU | (1<<LED_PIN1) | (1<<MULTITAP_PIN) | (1<<LATCH_PIN) | (1<<CLOCK_PIN); // 1 = output, 0 = input (if push-pull)
    P1_MOD_OC = P1_MOD_OC | ((1<<DATA0_PIN) | (1<<DATA1_PIN) | (1<<DATA2_PIN) | (1<<DATA3_PIN)); // Make them all open drain
    P1_DIR_PU = P1_DIR_PU | (1<<DATA0_PIN) | (1<<DATA1_PIN) | (1<<DATA2_PIN) | (1<<DATA3_PIN); // pull-up enable (if open drain)

    // clock is normally high
    SNES_CLOCK_HIGH();

    // LATCH is Active HIGH
    SNES_LATCH_LOW();

    ctrlFlags.nesMode = 0;
    if (ctrlFlags.live_autodetect == 0)
    {
        /* Snes controller buttons are sent in this order:
            * 1st byte: B Y SEL START UP DOWN LEFT RIGHT
            * 2nd byte: A X L R 1 1 1 1
            *
            * Nes controller buttons are sent in this order:
            * One byte: A B SEL START UP DOWN LEFT RIGHT
            *
            * When an additional byte is read from a NES controller,
            * all bits are 0. Because the data signal is active low,
            * this corresponds to pressed buttons. When we read
            * from the controller for the first time, detect NES
            * controllers by checking those 4 bits.
            **/
        ctrlFlags.live_autodetect = 1;
        fournsnesUpdate();
        ctrlFlags.live_autodetect = 0;
    }
    else
    {
        fournsnesUpdate();
    }

    autoDetectFourScore();
    autoDetectSNESMultiTap();
}

static void fournsnesUpdate_fourscore(void)
{
	int i;
	unsigned char tmp1=0;
	unsigned char tmp2=0;
	unsigned char tmp3=0;
	unsigned char tmp4=0;
	SNES_LATCH_HIGH();
	_delay_us(12);
	SNES_LATCH_LOW();

	/* Nes controller buttons are sent in this order:
	 * One byte: A B SEL START UP DOWN LEFT RIGHT */

	for (i=0; i<8; i++)
	{
		_delay_us(6);
		SNES_CLOCK_LOW();

		// FourScore to be connected to ports 1 and 2
		tmp1 <<= 1;
		tmp2 <<= 1;
		if (!SNES_GET_DATA1()) { tmp1 |= 1; }
		if (!SNES_GET_DATA2()) { tmp2 |= 1; }

		_delay_us(6);
		SNES_CLOCK_HIGH();
	}

	for (i=0; i<8; i++)
	{
		_delay_us(6);
		SNES_CLOCK_LOW();

		// FourScore to be connected to ports 1 and 2
		tmp3 <<= 1;
		tmp4 <<= 1;
		if (!SNES_GET_DATA1()) { tmp3 |= 1; }
		if (!SNES_GET_DATA2()) { tmp4 |= 1; }

		_delay_us(6);
		SNES_CLOCK_HIGH();
	}

	for (i=0; i<8; i++)
	{
		_delay_us(6);
		SNES_CLOCK_LOW();

		_delay_us(6);
		SNES_CLOCK_HIGH();
	}
	last_read_controller_bytes[0] = tmp1;
	last_read_controller_bytes[2] = tmp2;
	last_read_controller_bytes[4] = tmp3;
	last_read_controller_bytes[6] = tmp4;
}
/*
 *
       Clock Cycle     Button Reported
        ===========     ===============
        1               B
        2               Y
        3               Select
        4               Start
        5               Up on joypad
        6               Down on joypad
        7               Left on joypad
        8               Right on joypad
        9               A
        10              X
        11              L
        12              R
        13              none (always high)
        14              none (always high)
        15              none (always high)
        16              none (always high)
 *
 */

void fournsnesUpdate(void)
{
    int i;
    unsigned char tmp1=0;
    unsigned char tmp2=0;
    unsigned char tmp3=0;
    unsigned char tmp4=0;

    if (ctrlFlags.fourscore_mode)
  	{
  		fournsnesUpdate_fourscore();
  		return;
  	}


  	if (ctrlFlags.multitap_mode)
  	{
		SNES_LATCH_HIGH();
		_delay_us(12);
		SNES_LATCH_LOW();
		_delay_us(12);

		MTAP_SELECT_HIGH();
		_delay_us(6);
		for (i=0; i<8; i++)
		{
			SNES_CLOCK_LOW();
			_delay_us(6);

			tmp1 <<= 1;
			tmp2 <<= 1;

			if (!SNES_GET_DATA1()) { tmp1 |= 1; }
			if (!SNES_GET_DATA2()) { tmp2 |= 1; }

			SNES_CLOCK_HIGH();
			_delay_us(6);
		}
		last_read_controller_bytes[0] = tmp1;
		last_read_controller_bytes[2] = tmp2;
		for (i=0; i<8; i++)
		{
			SNES_CLOCK_LOW();
			_delay_us(6);

			tmp1 >>= 1;
			tmp2 >>= 1;

			if (!SNES_GET_DATA1()) { tmp1 |= 0x80; }
			if (!SNES_GET_DATA2()) { tmp2 |= 0x80; }

			SNES_CLOCK_HIGH();
			_delay_us(6);
		}


		MTAP_SELECT_LOW();
		_delay_us(6);
		for (i=0; i<8; i++)
		{
			SNES_CLOCK_LOW();
			_delay_us(6);

			tmp3 <<= 1;
			tmp4 <<= 1;

			if (!SNES_GET_DATA1()) { tmp3 |= 1; }
			if (!SNES_GET_DATA2()) { tmp4 |= 1; }

			SNES_CLOCK_HIGH();
			_delay_us(6);
		}
		last_read_controller_bytes[4] = tmp3;
		last_read_controller_bytes[6] = tmp4;
		for (i=0; i<8; i++)
		{
			SNES_CLOCK_LOW();
			_delay_us(6);

			tmp3 >>= 1;
			tmp4 >>= 1;

			if (!SNES_GET_DATA1()) { tmp3 |= 0x80; }
			if (!SNES_GET_DATA2()) { tmp4 |= 0x80; }

			SNES_CLOCK_HIGH();
			_delay_us(6);
		}
  	}
    else // standard mode (not multitap)
  	{
		SNES_LATCH_HIGH();
		_delay_us(12);
		SNES_LATCH_LOW();

		for (i=0; i<8; i++)
		{
			_delay_us(6);
			SNES_CLOCK_LOW();

			tmp1 <<= 1;
			if (!SNES_GET_DATA1()) { tmp1 |= 1; }
			tmp2 <<= 1;
			if (!SNES_GET_DATA2()) { tmp2 |= 1; }
#if NUM_GAMEPADS > 2
			tmp3 <<= 1;
			if (!SNES_GET_DATA3()) { tmp3 |= 1; }
			tmp4 <<= 1;
			if (!SNES_GET_DATA4()) { tmp4 |= 1; }
#endif
			_delay_us(6);
			SNES_CLOCK_HIGH();
		}
		last_read_controller_bytes[0] = tmp1;
		last_read_controller_bytes[2] = tmp2;
#if NUM_GAMEPADS > 2
		last_read_controller_bytes[4] = tmp3;
		last_read_controller_bytes[6] = tmp4;
#endif
		for (i=0; i<8; i++)
		{
			_delay_us(6);

			SNES_CLOCK_LOW();

			// notice that this is different from above. We
			// want the bits to be in reverse-order
			tmp1 >>= 1;
			if (!SNES_GET_DATA1()) { tmp1 |= 0x80; }
			tmp2 >>= 1;
			if (!SNES_GET_DATA2()) { tmp2 |= 0x80; }
#if NUM_GAMEPADS > 2
			tmp3 >>= 1;
			if (!SNES_GET_DATA3()) { tmp3 |= 0x80; }
			tmp4 >>= 1;
			if (!SNES_GET_DATA4()) { tmp4 |= 0x80; }
#endif
			_delay_us(6);
			SNES_CLOCK_HIGH();
		}
    }

    if (ctrlFlags.live_autodetect) {
        if (tmp1==0xFF)
            ctrlFlags.nesMode |= 1;
        else
            ctrlFlags.nesMode &= 0xFE;

        if (tmp2==0xFF)
            ctrlFlags.nesMode |= 2;
        else
            ctrlFlags.nesMode &= (0xFF ^ 2);

#if NUM_GAMEPADS > 2
        if (tmp3==0xFF)
            ctrlFlags.nesMode |= 4;
        else
            ctrlFlags.nesMode &= (0xFF ^ 4);

        if (tmp4==0xFF)
            ctrlFlags.nesMode |= 8;
        else
            ctrlFlags.nesMode &= (0xFF ^ 8);
#endif
    }
    /* Force extra bits to 0 when in NES mode. Otherwise, if
        * we read zeros on the wire, we will have permanantly
        * pressed buttons */
    last_read_controller_bytes[1] = (ctrlFlags.nesMode & 1) ? 0x00 : tmp1;
    last_read_controller_bytes[3] = (ctrlFlags.nesMode & 2) ? 0x00 : tmp2;
#if NUM_GAMEPADS > 2
    last_read_controller_bytes[5] = (ctrlFlags.nesMode & 4) ? 0x00 : tmp3;
    last_read_controller_bytes[7] = (ctrlFlags.nesMode & 8) ? 0x00 : tmp4;
#endif
}

unsigned char nesSnesGetX(unsigned char nesByte1)
{
    if (nesByte1&0x1) { return 255; }
    if (nesByte1&0x2) { return 0; }
    return 128;
}

unsigned char nesSnesGetY(unsigned char nesByte1)
{
    if (nesByte1&0x4) { return 255; }
    if (nesByte1&0x8) { return 0; }
    return 128;
}

/* Move the bits around so that identical NES and SNES buttons
 * use the same USB button IDs. */
unsigned char nesReorderButtons(unsigned char raw)
{
    unsigned char v;
    v = (raw & 0x80) >> 3;
    v |= (raw & 0x40) >> 6;
    v |= (raw & 0x20) >> 3;
    v |= (raw & 0x10) >> 1;
    return v;
}

unsigned char snesReorderButtons(unsigned char bytes[2])
{
    unsigned char v;

    /* pack the snes button bits, which are on two bytes, in
        * one single byte. */
    v =	(bytes[0]&0x80)>>7;
    v |= (bytes[0]&0x40)>>5;
    v |= (bytes[0]&0x20)>>3;
    v |= (bytes[0]&0x10)>>1;
    v |= (bytes[1]&0x0f)<<4;

    return v;
}

char fournsnesBuildReport(unsigned char *reportBuffer, unsigned char id)
{
	int idx;

	if ((id == 0) || id > 4)
		return 0;

	/* last_read_controller_bytes[] structure:
	 *
	 * [0] : controller 1, 8 first bits (dpad + start + sel + y|a + b)
	 * [1] : controller 1, 8 snes extra bits (4 lower bits are buttons)
	 *
	 * [2] : controller 2, 8 first bits
	 * [3] : controller 2, 4 extra snes buttons
	 *
	 * [4] : controller 3, 8 first bits
	 * [5] : controller 3, 4 extra snes buttons
	 *
	 * [6] : controller 4, 8 first bits
	 * [7] : controller 4, 4 extra snes buttons
	 *
	 *
	 * last_read_controller_bytes[] structure in FOUR SCORE mode:
	 *
	 *  A B SEL START UP DOWN LEFT RIGHT
	 *
	 * [0] : NES controller 1 data
	 * [1] : NES controller 2 data
	 * [2] : NES controller 3 data
	 * [3] : NES controller 4 data
	 *
	 */

	idx = id - 1;
	if (reportBuffer != NULL)
	{
        // Don't need to set this each time -- it's already set in main()
		//reportBuffer[0]=id;
		reportBuffer[1]=nesSnesGetX(last_read_controller_bytes[idx*2]);
		reportBuffer[2]=nesSnesGetY(last_read_controller_bytes[idx*2]);

		if (ctrlFlags.nesMode & (0x01<<idx))
		{
			reportBuffer[3] = nesReorderButtons(last_read_controller_bytes[idx*2]);
			#if NUM_BUTTONS > 8
			reportBuffer[4] = 0;
			#endif
		}
		else {
			reportBuffer[3] = snesReorderButtons(&last_read_controller_bytes[idx*2]);
			#if NUM_BUTTONS > 8
			reportBuffer[4] = (last_read_controller_bytes[(idx*2)+1] & 0xF0) >> 4;
			#endif
		}
	}
	return GAMEPAD_XMIT_DATA_LEN;
}
