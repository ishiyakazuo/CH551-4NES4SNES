/* Name: fourplay.c
 * Project: Multiple PSX to USB converter
 * Author: Jeff Stenhouse (structured around Raphael Assenat's 4nes4snes code)
 * Copyright: (C) 2020 Jeff Stenhouse
 * License: GPLv2
 * Tabsize: 4
 */
#include "fourplay.h"
#include "ch554.h"
#include "debug.h"
#include <stdio.h>
#include <string.h>

// DualShock controllers have 9 bytes we care about.
#define NUM_READ_BYTES_PER_GAMEPAD  9
#define GAMEPAD_BYTES	(NUM_GAMEPADS*NUM_READ_BYTES_PER_GAMEPAD)

/******** IO port definitions **************/
#define PORT1REG 0x90
#define PORT2REG 0xA0
#define PORT3REG 0xB0

// PSX controller pinout:
// 1 = Data (from controller) - this is a SPI bus, open collector. Chip select determines which controller is being read
// 2 = Command (from host) - one signal goes to every controller
// 3 = ~+8V (not used here, for DualShock)
// 4 = GND
// 5 = Vcc (+3.3V to be safe)
// 6 = Device select/SPI Chip Select (from host) - one per controller needed
// 7 = Clock (from host) - ideally 250-500kHz
// 8 = N/C
// 9 = ACK (from controller) - this is open collector and can go to all controller ports.
// Total pins needed: 6 common pins + 1 per controller (chip select)

// Outputs from host are on port 3, inputs from controller are on port 1.

#define LED_PIN1 0 // P3.0

#define DATA_PIN 1 // P3.1
SBIT(DATA_SIG, PORT3REG, DATA_PIN);
#define CMD_PIN 2 // P3.2
SBIT(CMD_SIG, PORT3REG, CMD_PIN);
#define CLOCK_PIN 3 // P3.3
SBIT(CLOCK_SIG, PORT3REG, CLOCK_PIN);

#define ACK_PIN 1 // P1.1
SBIT(ACK_SIG, PORT1REG, ACK_PIN);
#define CHIPSEL1_PIN 4 // P1.4
SBIT(CHIPSEL1_SIG, PORT1REG, CHIPSEL1_PIN);
#define CHIPSEL2_PIN 5 // P1.5
SBIT(CHIPSEL2_SIG, PORT1REG, CHIPSEL2_PIN);
#define CHIPSEL3_PIN 6 // P1.6
SBIT(CHIPSEL3_SIG, PORT1REG, CHIPSEL3_PIN);
#define CHIPSEL4_PIN 7 // P1.7
SBIT(CHIPSEL4_SIG, PORT1REG, CHIPSEL4_PIN);

#define _delay_us mDelayuS

/********* IO port manipulation macros **********/
#define PSX_CLOCK_LOW()  CLOCK_SIG = 0
#define PSX_CLOCK_HIGH() CLOCK_SIG = 1
#define PSX_CMD_LOW()    CMD_SIG = 0
#define PSX_CMD_HIGH()   CMD_SIG = 1

#define PSX_GET_DATA()	DATA_SIG
#define PSX_GET_ACK()	DATA_SIG

#define SET_CHIP_SEL_LOW(a)  P1 &= ~(1 << (CHIPSEL1_PIN+a))
#define SET_CHIP_SEL_HIGH(a) P1 |= (1 << (CHIPSEL1_PIN+a))

#define INVERT_BYTE(a) (a ^ 0xFF)

// First byte
#define PSB_SELECT      0x01
#define PSB_L3          0x02
#define PSB_R3          0x04
#define PSB_START       0x08
#define PSB_PAD_UP      0x10
#define PSB_PAD_RIGHT   0x20
#define PSB_PAD_DOWN    0x40
#define PSB_PAD_LEFT    0x80
// Second byte
#define PSB_L2          0x01
#define PSB_R2          0x02
#define PSB_L1          0x04
#define PSB_R1          0x08
#define PSB_TRIANGLE    0x10
#define PSB_CIRCLE      0x20
#define PSB_CROSS       0x40
#define PSB_SQUARE      0x80

/*********** prototypes *************/
// the most recent bytes we fetched from the controller
static __xdata uint8_t last_read_controller_bytes[GAMEPAD_BYTES];
static __xdata uint8_t junk_return_data[16];

static uint16_t incrementer = 0;

__code uint8_t enter_config[]={0x01,0x43,0x00,0x01,0x00};
__code uint8_t set_mode[]={0x01,0x44,0x00,0x01,0x03,0x00,0x00,0x00,0x00};
__code uint8_t exit_config[]={0x01,0x43,0x00,0x00,0x5A,0x5A,0x5A,0x5A,0x5A};
//__code uint8_t type_read[]={0x01,0x45,0x00,0x5A,0x5A,0x5A,0x5A,0x5A,0x5A};

__code uint8_t psxPollingCommand[9] = {
	0x01, // Always 0x01 for controller
    0x42, // Controller poll command
    0x00,
    0x00, // Motor #1 strength
    0x00, // Motor #2 strength
    0x00,
    0x00,
    0x00,
    0x00
};

typedef struct controllerFlags {
    unsigned dualShockMode   : 4; // Don't know if we need this yet...
    unsigned multitap_mode   : 1; // TODO, maybe.
    unsigned live_autodetect : 1;
} controllerFlags;

static struct controllerFlags ctrlFlags;

#if CONTROLLER_TYPE_PSX
void disableLiveAutodetect(void)
{
    ctrlFlags.live_autodetect = 0;
}

void enableLiveAutodetect(void)
{
    ctrlFlags.live_autodetect = 1;
}
#endif
static void autoDetectPSXMultiTap(void)
{
// TODO
}

void fourplaySetAnalogMode(uint8_t port);

void fourplayInit(void)
{
	// It just so happens that all of our port 3 pins are push/pull outputs, and all of our port 1 pins are inputs w/ pull-ups...
    P3_MOD_OC = P3_MOD_OC & ~((1<<LED_PIN1) | (1<<CMD_PIN) | (1<<CLOCK_PIN)); // 0 = push/pull, 1 = open-drain
    P3_MOD_OC = P3_MOD_OC | (1<<DATA_PIN); // 0 = push/pull, 1 = open-drain (DATA is the only open-drain one)
    P3_DIR_PU = P3_DIR_PU | (1<<LED_PIN1) | (1<<DATA_PIN) | (1<<CMD_PIN) | (1<<CLOCK_PIN); // 1 = output, 0 = input (if push-pull)
    P1_MOD_OC = P1_MOD_OC | (1<<ACK_PIN); // Make ACK open-drain, and chip selects push/pull.
    P1_DIR_PU = P1_DIR_PU | (1<<ACK_PIN) | (1<<CHIPSEL1_PIN) | (1<<CHIPSEL2_PIN) | (1<<CHIPSEL3_PIN) | (1<<CHIPSEL4_PIN); // pull-up enable for ACK, output mode for chip selects.

    // clock is normally high
    PSX_CLOCK_HIGH();

    ctrlFlags.dualShockMode = 0;
    if (ctrlFlags.live_autodetect == 0)
    {
        for (int i = 0; i < NUM_GAMEPADS; i++)
        {
            fourplaySetAnalogMode(i);
        }
    }
    fourplayUpdate();
    autoDetectPSXMultiTap();
}

uint8_t fourplayCmdByte(uint8_t cmdByteOut)
{
    uint8_t tmpByteIn = 0; // Not sure we need this...
    for (uint8_t bitNum = 0; bitNum < 8; bitNum++)
    {
        PSX_CLOCK_LOW();
        if (cmdByteOut & 0x01)  { PSX_CMD_HIGH(); }
        else                    { PSX_CMD_LOW(); }
        cmdByteOut >>= 1;
        _delay_us(CLOCK_DELAY_US);
        PSX_CLOCK_HIGH();
        tmpByteIn >>= 1;
        if (PSX_GET_DATA()) { tmpByteIn |= 0x80; }
        _delay_us(CLOCK_DELAY_US);
    }
    return tmpByteIn;
}

uint8_t fourplayWaitAck()
{
    uint8_t ackWait = 100; // Timeout after approximately 100us.
    while (ackWait)
    {
        ackWait--;
        if (PSX_GET_ACK() == 0)
        {
            while (ackWait && (PSX_GET_ACK() == 0))
            {
                ackWait--;
                _delay_us(1);
            }
            return 1; // Received an ACK, and now moving on with life.
        }
        _delay_us(1);
    }
    return 0;
}

void fourplayCmd(uint8_t* cmd, uint8_t* resp, uint8_t len, uint8_t ackByteNum)
{
    uint8_t byteNum, tmpByteIn;
    for (byteNum = 0; byteNum < len; byteNum++)
    {
        tmpByteIn = fourplayCmdByte(cmd[byteNum]);
        if (byteNum < ackByteNum)
        {
            if (!fourplayWaitAck())
            {
                //tmpByteIn = 0;
            }
        }
        resp[byteNum] = tmpByteIn;
    }
}

void fourplaySetAnalogMode(uint8_t port)
{
  SET_CHIP_SEL_LOW(port);
  fourplayCmd(enter_config, junk_return_data, sizeof(enter_config), sizeof(enter_config)-1);
  SET_CHIP_SEL_HIGH(port);
  _delay_us(10);
  SET_CHIP_SEL_LOW(port);
  fourplayCmd(set_mode, junk_return_data, sizeof(set_mode), sizeof(set_mode)-1);
  SET_CHIP_SEL_HIGH(port);
  _delay_us(10);
  SET_CHIP_SEL_LOW(port);
  fourplayCmd(exit_config, junk_return_data, sizeof(exit_config), sizeof(exit_config)-1);
  SET_CHIP_SEL_HIGH(port);
}

void fourplayUpdate(void)
{
    int i;
    uint8_t controllerOffset;
    uint8_t controllerType = 0;
    for (i = 0; i < NUM_GAMEPADS; i++)
    {
        if (ctrlFlags.live_autodetect)
        {
            fourplaySetAnalogMode(i);
        }
        controllerOffset = NUM_READ_BYTES_PER_GAMEPAD*i;
        SET_CHIP_SEL_LOW(i);
        fourplayCmd(psxPollingCommand, &last_read_controller_bytes[controllerOffset], 3, 3);
        controllerType = last_read_controller_bytes[controllerOffset+1];
        if ((controllerType == 0xFF) || ((controllerType & 0x40) != 0x40))
        { // No controller connected, or unsupported type (e.g. NegCon)
            memset(&last_read_controller_bytes[controllerOffset], 0xFF, 5);
            memset(&last_read_controller_bytes[controllerOffset+5], 0x7F, 4);
        }
        else
        {
            if (controllerType & 0x10)
            {
                fourplayCmd(&psxPollingCommand[3], &last_read_controller_bytes[controllerOffset+3], 6, 5);
                if (controllerType == 0x79)
                {
                    for (int j = 0; j < 10; j++)
                    {
                        fourplayWaitAck();
                        fourplayCmdByte(0);
                    }
                    // Get L2 and R2, and store them into two bytes that aren't otherwise very useful.
                    fourplayWaitAck();
                    last_read_controller_bytes[0] = fourplayCmdByte(0); // L2
                    fourplayWaitAck();
                    last_read_controller_bytes[2] = fourplayCmdByte(0); // R2
                }
            }
            else
            {
                fourplayCmd(&psxPollingCommand[3], &last_read_controller_bytes[controllerOffset+3], 2, 1);
                memset(&last_read_controller_bytes[controllerOffset+5], 0x7F, 4);
            }
        }
        SET_CHIP_SEL_HIGH(i);
    }
}

unsigned char psxGetX(unsigned char byte1)
{
    if ((byte1 & PSB_PAD_LEFT) == 0)  { return 0;   }
    if ((byte1 & PSB_PAD_RIGHT) == 0) { return 255; }
    return 128;
}

unsigned char psxGetY(unsigned char byte1)
{
    if ((byte1 & PSB_PAD_DOWN) == 0) { return 255; }
    if ((byte1 & PSB_PAD_UP) == 0)   { return 0; }
    return 128;
}

// Maps the buttons to the Xbox 360 mapping, as it's the de-facto controller mapping in many games
unsigned char psxGetButtonByte1(unsigned char bytes[2])
{
    unsigned char retVal;
    retVal  = (bytes[1] & PSB_CROSS)    ? 0 : 0x01;
    retVal |= (bytes[1] & PSB_CIRCLE)   ? 0 : 0x02;
    retVal |= (bytes[1] & PSB_SQUARE)   ? 0 : 0x04;
    retVal |= (bytes[1] & PSB_TRIANGLE) ? 0 : 0x08;
    retVal |= (bytes[1] & PSB_L1)       ? 0 : 0x10;
    retVal |= (bytes[1] & PSB_R1)       ? 0 : 0x20;
    retVal |= (bytes[0] & PSB_SELECT)   ? 0 : 0x40;
    retVal |= (bytes[0] & PSB_START)    ? 0 : 0x80;
    return retVal;
}

unsigned char psxGetButtonByte2(unsigned char bytes[2])
{
    unsigned char retVal;
    retVal  = (bytes[0] & PSB_L3) ? 0 : 0x02;
    retVal |= (bytes[0] & PSB_R3) ? 0 : 0x04;
    retVal |= (bytes[1] & PSB_L2) ? 0 : 0x08;
    retVal |= (bytes[1] & PSB_R2) ? 0 : 0x10;
    return retVal;
}

char fourplayBuildReport(unsigned char *reportBuffer, unsigned char id)
{
	int idx, offset;

	if ((id == 0) || id > 4)
		return 0;

	idx = id - 1;
	if (reportBuffer != NULL)
	{
        // Don't need to set this each time -- it's already set in main()
		//reportBuffer[0]=id;
        offset = (idx*NUM_READ_BYTES_PER_GAMEPAD);
		reportBuffer[1]=psxGetX(last_read_controller_bytes[offset+3]);
		reportBuffer[2]=psxGetY(last_read_controller_bytes[offset+3]);
        reportBuffer[3]=psxGetButtonByte1(&last_read_controller_bytes[offset+3]);
        reportBuffer[4]=psxGetButtonByte2(&last_read_controller_bytes[offset+3]);
        #if (NUM_AXES > 2)
        // TODO: sticks and analog buttons
        if (last_read_controller_bytes[offset+1] == 0x79)
        {
            // Can use analog L2/R2
        }
        #endif
	}
	return GAMEPAD_XMIT_DATA_LEN;
}
