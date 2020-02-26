#ifndef __FOURPLAY
#define __FOURPLAY

// Maximum number of supported gamepads is 4 (only 4 endpoints available)
#ifndef NUM_GAMEPADS
#define NUM_GAMEPADS 4
#endif
// DualShock 2 has 12 buttons, but we're mapping it to an Xbox 360 controller,
// which has 11 + 2 analog.  I'm leaving the option to use L2/R2 as digital, so 13.
#ifndef NUM_BUTTONS
#define NUM_BUTTONS 13
#endif

#ifndef NUM_AXES
#define NUM_AXES 8
#endif

#if (NUM_AXES & 1)
#error You cannot have an odd number of axes defined.
#endif
#if (NUM_AXES < 2)
#error You need to define a minimum of 2 axes.
#endif
#if (NUM_AXES > 8)
#error You cannot define more than 8 axes.
#endif

#ifndef CLOCK_DELAY_US
#define CLOCK_DELAY_US 4
#endif

/*
#define XY_AXIS_COL_LENGTH 18*(NUM_AXES/2)
#define XY_AXIS_COLLECTION(a,b)  0xa1, 0x00, 0x09, a, 0x09, b, \
    0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, 0x02, 0x81, 0x02, 0xC0,

#define AXIS_DESCR1 XY_AXIS_COLLECTION(0x30, 0x31)
#if (NUM_AXES >= 4)
#define AXIS_DESCR2 XY_AXIS_COLLECTION(0x32, 0x33)
#else
#define AXIS_DESCR2
#endif
#if (NUM_AXES >= 6)
#define AXIS_DESCR3 XY_AXIS_COLLECTION(0x34, 0x35)
#else
#define AXIS_DESCR3
#endif
#if (NUM_AXES >= 8)
#define AXIS_DESCR4 XY_AXIS_COLLECTION(0x36, 0x37)
#else
#define AXIS_DESCR4
#endif

#define AXIS_COLLECTIONS AXIS_DESCR1 AXIS_DESCR2 AXIS_DESCR3 AXIS_DESCR4
*/
#define XY_AXIS_COL_LENGTH 14+(NUM_AXES*2)
#define XY_AXIS_COLLECTION(a,b) 0x09, a, 0x09, b

#define AXIS_DESCR1 XY_AXIS_COLLECTION(0x30, 0x31),
#if (NUM_AXES >= 4)
#define AXIS_DESCR2 XY_AXIS_COLLECTION(0x32, 0x33),
#else
#define AXIS_DESCR2
#endif
#if (NUM_AXES >= 6)
#define AXIS_DESCR3 XY_AXIS_COLLECTION(0x34, 0x35),
#else
#define AXIS_DESCR3
#endif
#if (NUM_AXES >= 8)
#define AXIS_DESCR4 XY_AXIS_COLLECTION(0x36, 0x37),
#else
#define AXIS_DESCR4
#endif

#define AXIS_COLLECTIONS  0xa1, 0x00, AXIS_DESCR1 AXIS_DESCR2 AXIS_DESCR3 AXIS_DESCR4 \
    0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, NUM_AXES, 0x81, 0x02, 0xC0,


// This is the PSX controller's report descriptor.  Defined as follows:
/*
0x05, 0x01, \			// USAGE_PAGE (Generic Desktop)
0x09, 0x04, \			// USAGE (Joystick)
0xa1, 0x01, \			//	COLLECTION (Application)
0x85, a, \			  //			REPORT_ID (a)
  0x09, 0x01, \			//		USAGE (Pointer)

  // This is the section above (XY_AXIS_COLLECTION).
  0xa1, 0x00, \			//		COLLECTION (Physical)
    0x09, 0x30, \			  //			USAGE (X)
    0x09, 0x31, \			  //			USAGE (Y)
    0x15, 0x00, \			  //			LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00, \	//		  LOGICAL_MAXIMUM (255)
    0x75, 0x08, \			  //			REPORT_SIZE (8)
    0x95, 0x02, \			  //			REPORT_COUNT (2)
    0x81, 0x02, \			  //			INPUT (Data,Var,Abs)

    0x05, 0x09, \			  //			USAGE_PAGE (Button)
    0x19, 1, \			    //   		USAGE_MINIMUM (Button 1)
    0x29, NUM_BUTTONS, \			    //   		USAGE_MAXIMUM (Button X = NUM_BUTTONS)
    0x15, 0x00, \			  //   		LOGICAL_MINIMUM (0)
    0x25, 0x01, \			  //   		LOGICAL_MAXIMUM (1)
    0x75, 1, \			    // 			REPORT_SIZE (1)
    0x95, NUM_BUTTONS, \			    //			REPORT_COUNT (NUM_BUTTONS)
    0x81, 0x02, \			  //			INPUT (Data,Var,Abs)

// It might have the following, if the number of buttons isn't a multiple of 8
// (Windows requires byte alignment.)
    0x95, X, \			    //			REPORT_COUNT (X)
    0x81, 0x03, \			  //			INPUT (Const,Var,Abs)

  0xc0, \				//		END_COLLECTION
0xc0 // END_COLLECTION
*/

#if (NUM_BUTTONS % 8) // Need to add byte alignment
#define BYTE_ALIGNMENT_DESC_LEN 4
#define BUTTON_BYTE_ALIGNMENT 0x95,(8-(NUM_BUTTONS%8)),0x81,0x03,
#else
#define BYTE_ALIGNMENT_DESC_LEN 0
#define BUTTON_BYTE_ALIGNMENT
#endif

#define REP_DESC_LEN (27+(XY_AXIS_COL_LENGTH)+BYTE_ALIGNMENT_DESC_LEN)

#define GAMEPAD_REPORT_DESCRIPTOR(a) 0x05,0x01,0x09,0x04, \
    0xA1,0x01, \
        0x85,a, \
        0x09,0x01, \
        AXIS_COLLECTIONS \
			0x05,0x09,0x19,0x01,0x29,NUM_BUTTONS,0x15,0x00,0x25,0x01,0x75,0x01, \
            0x95,NUM_BUTTONS,0x81,0x02,BUTTON_BYTE_ALIGNMENT \
	0xC0

// This defines the # of bytes transmitted to the host per controller.
#define GAMEPAD_XMIT_DATA_LEN (1+NUM_AXES+((NUM_BUTTONS+7)/8))

#define CONTROLLER_INIT         fourplayInit
#define CONTROLLER_UPDATE       fourplayUpdate
#define BUILD_CONTROLLER_REPORT fourplayBuildReport

void fourplayInit(void);
void fourplayUpdate(void);
char fourplayBuildReport(unsigned char *reportBuffer, unsigned char report_id);
void enableLiveAutodetect(void);
void disableLiveAutodetect(void);

#endif
