typedef unsigned char*         PUINT8;
typedef unsigned char volatile UINT8V;

#include "ch554.h"
#include "debug.h"
#include <stdio.h>
#include <string.h>
#include <ch554_usb.h>

#if CONTROLLER_TYPE_SNES
  #include "fournsnes.h"
  __code uint8_t ProductName[] = {
      20, 0x03, 	// Length = 20 bytes, String Descriptor (0x03)
      '0'+NUM_GAMEPADS, 0,
      'N', 0,
      'E', 0,
      'S', 0,
      '0'+NUM_GAMEPADS, 0,
      'S', 0,
      'N', 0,
      'E', 0,
      'S', 0
  };

  // String Descriptors:
  __code uint8_t DevName1[] = {
      34, 0x03, 	// Length = 34 bytes, String Descriptor (0x03)
      'N', 0,
      'E', 0,
      'S', 0,
      '/', 0,
      'S', 0,
      'N', 0,
      'E', 0,
      'S', 0,
      ' ', 0,
      'G', 0,
      'a', 0,
      'm', 0,
      'e', 0,
      'p', 0,
      'a', 0,
      'd', 0
  };

  __code uint8_t ManuName[] = {
      16, 0x03, 	// Length = 30 bytes, String Descriptor (0x03)
      'R', 0,
      'a', 0,
      'p', 0,
      'h', 0,
      'n', 0,
      'e', 0,
      't', 0
  };
#endif

#if CONTROLLER_TYPE_PSX
  #include "fourplay.h"
__code unsigned char ProductName[] = {
    20, 0x03, 	// Length = 20 bytes, String Descriptor (0x03)
    '0'+NUM_GAMEPADS, 0,
    'P', 0,
    'l', 0,
    'a', 0,
    'y', 0,
    ' ', 0,
    'P', 0,
    'a', 0,
    'd', 0
};

// String Descriptors:
__code unsigned char DevName1[] = {
    24, 0x03, 	// Length = 24 bytes, String Descriptor (0x03)
    'P', 0,
    'S', 0,
    'X', 0,
    ' ', 0,
    'G', 0,
    'a', 0,
    'm', 0,
    'e', 0,
    'p', 0,
    'a', 0,
    'd', 0
};

__code unsigned char ManuName[] = {
    20, 0x03, 	// Length = 20 bytes, String Descriptor (0x03)
    'H', 0,
    'a', 0,
    'r', 0,
    'p', 0,
    's', 0,
    't', 0,
    'o', 0,
    'n', 0,
    'e', 0
};
#endif


uint32_t millis, last;

#define PORT1REG 0x90
#define PORT2REG 0xA0
#define PORT3REG 0xB0

#define LED_PIN1 0
SBIT(LED1, PORT3REG, LED_PIN1);

#define MULTITAP_PIN 1
#define LATCH_PIN 2 // P3.2
#define CLOCK_PIN 3 // P3.3
#define DATA0_PIN 4 // P1.4
#define DATA1_PIN 5 // P1.5
#define DATA2_PIN 6 // P1.6
#define DATA3_PIN 7 // P1.7

__xdata __at (0x0000) uint8_t  Ep0Buffer[64];
__xdata __at (0x0040) uint8_t  Ep4Buffer[64];
__xdata __at (0x0080) uint8_t  Ep1Buffer[64];
__xdata __at (0x00C0) uint8_t  Ep2Buffer[64];
__xdata __at (0x0100) uint8_t  Ep3Buffer[64];

uint8_t SetupReq,SetupLen,Ready,Count,UsbConfig;
PUINT8  pDescr;
USB_SETUP_REQ   SetupReqBuf;

void mTimer0Interrupt( void ) __interrupt (INT_NO_TMR0)
{
	TH0 = (65536 - 2000)/256;  // Reload
	TL0 = (65536 - 2000)%256;  // Reload
	millis++;
}

#define	BIT0 (0X01)
#define	BIT1 (0X02)
#define	BIT2 (0X04)
#define	BIT3 (0X08)
#define	BIT4 (0X10)
#define	BIT5 (0X20)
#define	BIT6 (0X40)
#define	BIT7 (0X80)

#define UsbSetupBuf     ((PUSB_SETUP_REQ)Ep0Buffer)

__code uint8_t DevDesc[18] = {
	18, // length (18)
	0x01, // Descriptor type
	0x10,0x01, // USB version (1.1)
	0x00, // class
	0x00, // subclass
	0x00, // protocol
	0x08, // Max packet size
	0x50,0x1D, // VID 0x1D50 (OpenMoko)
	0x2D,0x60, // PID 0x602D (5nes5snes (4x12))
	0x00,0x00, // Device version
	0x03, // manufacturer string index
	0x02, // product string index
	0x00, // S/N string index
	0x01 // # of configurations
};

// Decoder ring for below descriptors
/*
9,          // sizeof(usbDescrInterface): length of descriptor in bytes
0x04, // descriptor type
0,          // index of this interface
0,          // alternate setting for this interface
1,   // endpoints excl 0: number of endpoint descriptors to follow
3, // class
0, // subclass
0, // protocol
1,          // string index for interface

9,          // sizeof(usbDescrHID): length of descriptor in bytes
0x21,   // descriptor type: HID
0x11, 0x01, // BCD representation of HID version
0x00,       // target country code
0x01,       // number of HID Report (or other HID class) Descriptor infos to follow
0x22,       // descriptor type: report
REP_DESC_LEN, 0,  // total length of report descriptor

7,          // sizeof(usbDescrEndpoint)
5,  // descriptor type = endpoint
0x81,       // IN endpoint number 1
0x03,       // attrib: Interrupt endpoint
16, 0,       // maximum packet size
10, // in ms
*/
// poll interval of host in milliseconds
#define POLL_INTERVAL 2

#define CFG_DESC_LEN (9+((9+9+7)*NUM_GAMEPADS))
#define CFG_INTERFACE_DESCR(a)  0x09,0x04,a,0x00,0x01,0x03,0x00,0x00,0x01
#define CFG_HID_DESCR  0x09,0x21,0x11,0x01,0x00,0x01,0x22,REP_DESC_LEN,0x00
#define CFG_EP_DESCR(a) 0x07,0x05,a,0x03,0x10,0x00,POLL_INTERVAL

__code uint8_t CfgDesc[CFG_DESC_LEN] =
{
    9,   /* sizeof(usbDescriptorConfiguration): length of descriptor in bytes */
    0x02,    /* descriptor type */
    CFG_DESC_LEN,
    0,       /* total length of data returned (including inlined descriptors) */
    NUM_GAMEPADS,          /* number of interfaces in this configuration */
    1,          /* index of this configuration */
    0,          /* configuration name string index */
    0xA0, // USBATTR_BUSPOWER + USBATTR_REMOTEWAKE
    100/2,            /* max USB current in 2mA units */
/* interface descriptor follows inline: */

    CFG_INTERFACE_DESCR(0),
    CFG_HID_DESCR,
    CFG_EP_DESCR(0x81),
#if NUM_GAMEPADS > 1
    CFG_INTERFACE_DESCR(1),
    CFG_HID_DESCR,
    CFG_EP_DESCR(0x82),
#endif
#if NUM_GAMEPADS > 2
    CFG_INTERFACE_DESCR(2),
    CFG_HID_DESCR,
    CFG_EP_DESCR(0x83),
#endif
#if NUM_GAMEPADS > 3
    CFG_INTERFACE_DESCR(3),
    CFG_HID_DESCR,
    CFG_EP_DESCR(0x84)
#endif
};

// These descriptors are absolutely identical except for the report ID field, hence the #define above.
__code uint8_t ControllerRepDesc[NUM_GAMEPADS][REP_DESC_LEN] = {
  {GAMEPAD_REPORT_DESCRIPTOR(1)},
  #if NUM_GAMEPADS > 1
  {GAMEPAD_REPORT_DESCRIPTOR(2)},
  #endif
  #if NUM_GAMEPADS > 2
  {GAMEPAD_REPORT_DESCRIPTOR(3)},
  #endif
  #if NUM_GAMEPADS > 3
  {GAMEPAD_REPORT_DESCRIPTOR(4)}
  #endif
};

// Language Descriptor
__code uint8_t LangDesc[] = {
	4, 0x03,		// Length = 4 bytes, String Descriptor (0x03)
	0x09, 0x04	// 0x0409 English - United States
};

#define USB_STRINGDESC_COUNT 4
__code uint8_t* StringDescs[USB_STRINGDESC_COUNT] = {
	LangDesc,		// 0 (If you want to support string descriptors, you must have this!)
	DevName1,		// 1
	ProductName,	// 2
	ManuName		// 3
};

// Buffers that the gamepad data gets written into before it's sent to the USB endpoints
uint8_t HIDCtrl[NUM_GAMEPADS][GAMEPAD_XMIT_DATA_LEN];

void CH554SoftReset( )
{
  SAFE_MOD = 0x55;
  SAFE_MOD = 0xAA;
  GLOBAL_CFG |= bSW_RESET;
}

void CH554USBDevWakeup( )
{
  UDEV_CTRL |= bUD_LOW_SPEED;
  mDelaymS(2);
  UDEV_CTRL &= ~bUD_LOW_SPEED;
}

void USBDeviceInit()
{
    IE_USB = 0;
    USB_CTRL = 0x00; // USB device mode
    UEP2_DMA = (uint16_t)Ep2Buffer; // Set endpoint 2's DMA buffer to Ep2Buffer
    UEP3_DMA = (uint16_t)Ep3Buffer; // Set endpoint 3's DMA buffer to Ep3Buffer
    UEP2_3_MOD = bUEP3_TX_EN | bUEP2_TX_EN; // Set both endpoints 2 and 3 as IN endpoints
    UEP2_CTRL = UEP_T_RES_NAK;  // Tell host we don't have anything (yet)
    UEP3_CTRL = UEP_T_RES_NAK;  // Tell host we don't have anything (yet)
    UEP0_DMA = (uint16_t)Ep0Buffer;  // Set endpoint 0's DMA buffer to Ep0Buffer
									 // (WARNING: Ep4Buffer is always set to Ep0Buffer+0x40)
    UEP1_DMA = (uint16_t)Ep1Buffer;  // Set endpoint 1's DMA buffer to Ep3Buffer
    UEP4_1_MOD = bUEP1_TX_EN | bUEP4_TX_EN; // Set both endpoints 1 and 4 as IN endpoints
    UEP1_CTRL = UEP_T_RES_NAK; // Tell host we don't have anything (yet)
    UEP4_CTRL = UEP_T_RES_NAK; // Tell host we don't have anything (yet)
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK; // Tell host we don't have anything (yet) to transmit, but ready to receive

	USB_DEV_AD = 0x00;
	UDEV_CTRL = bUD_PD_DIS;                                   // Disable DP / DM pull-down resistor
	USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;     // Start USB device and DMA,
				// and automatically respond with NAK before interrupt flag is not cleared during interrupt
	UDEV_CTRL |= bUD_PORT_EN;                                 // Enable USB port
	USB_INT_FG = 0xFF;                                        // Clear all interrupt flags־
	USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST; // Enable suspend, transfer and bus reset interrupts
	IE_USB = 1;
}

void Enp1IntIn( )
{
    memcpy( Ep1Buffer, HIDCtrl[0], GAMEPAD_XMIT_DATA_LEN);      // Copy the last generated data to the endpoint
    UEP1_T_LEN = GAMEPAD_XMIT_DATA_LEN;                       // Let the Host know we have this many bytes to send
    UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK; // Enable acknowledgements
}
#if NUM_GAMEPADS > 1
void Enp2IntIn( )
{
    memcpy( Ep2Buffer, HIDCtrl[1], GAMEPAD_XMIT_DATA_LEN);
    UEP2_T_LEN = GAMEPAD_XMIT_DATA_LEN;
    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
}
#endif
#if NUM_GAMEPADS > 2
void Enp3IntIn( )
{
    memcpy( Ep3Buffer, HIDCtrl[2], GAMEPAD_XMIT_DATA_LEN);
    UEP3_T_LEN = GAMEPAD_XMIT_DATA_LEN;
    UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
}
#endif
#if NUM_GAMEPADS > 3
void Enp4IntIn( )
{
    memcpy( Ep4Buffer, HIDCtrl[3], GAMEPAD_XMIT_DATA_LEN);
    UEP4_T_LEN = GAMEPAD_XMIT_DATA_LEN;
    UEP4_CTRL = UEP4_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
}
#endif
void HIDValueHandle(int gamepadID)
{
	// Copy the freshest data to the endpoints and alert the Host.
    switch (gamepadID)
    {
        case 0:
	        Enp1IntIn();
            break;
    #if NUM_GAMEPADS > 1
        case 1:
           	Enp2IntIn();
            break;
    #endif
    #if NUM_GAMEPADS > 2
        case 2:
        	Enp3IntIn();
            break;
    #endif
    #if NUM_GAMEPADS > 3
        case 3:
        	Enp4IntIn();
            break;
    #endif
    }
}

void GamepadGetLatest()
{
    int i;
	CONTROLLER_UPDATE(); // polls pads

	// Each of these converts the (S)NES controller data into the HID data
	// in the format that our report descriptor specifies.
    for (i = 0; i < NUM_GAMEPADS; i++)
    {
    	BUILD_CONTROLLER_REPORT(HIDCtrl[i], i+1);
    }
}

void DeviceInterrupt(void) __interrupt (INT_NO_USB)
{
    uint8_t len = 0;
    while(UIF_TRANSFER)                                                            // USB transfer complete flag־
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
        case UIS_TOKEN_IN | 4:                                                  // Endpoint 4 interrupt endpoint upload
			UEP4_T_LEN = 0;                                                     // Clear the length (don't send data)
			UEP4_CTRL = UEP4_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           // NACK requests
			UEP4_CTRL ^= bUEP_T_TOG;  // Toggle DATA0/DATA1 so that the next message we send is known to be fresh
				// NOTE: Endpoint 4 on CH55x MUST toggle the DATA0/DATA1 -- it doesn't support auto-toggle.
			break;
        case UIS_TOKEN_IN | 3:                                                  // Endpoint 3 interrupt endpoint upload
			UEP3_T_LEN = 0;
			UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;
			UEP3_CTRL ^= bUEP_T_TOG;  // Don't need to actually do this on Endpoints 1-3,
				                      // but for consistency, I turned off auto-toggle.
			break;
        case UIS_TOKEN_IN | 2:                                                  // Endpoint 2 interrupt endpoint upload
			UEP2_T_LEN = 0;
			UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;
			UEP2_CTRL ^= bUEP_T_TOG;
			break;
        case UIS_TOKEN_IN | 1:                                                  // Endpoint 1 interrupt endpoint upload
			UEP1_T_LEN = 0;
			UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;
			UEP1_CTRL ^= bUEP_T_TOG;
			break;
        case UIS_TOKEN_SETUP | 0:                                                // Setup transaction
            len = USB_RX_LEN;
            if(len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = UsbSetupBuf->wLengthL;
                if(UsbSetupBuf->wLengthH || SetupLen > 0x7F )
                {
                    SetupLen = 0x7F;    // Limit the total length to the maximum
                }
                len = 0;                                                        // Default is successful with 0 length
                SetupReq = UsbSetupBuf->bRequest;
                if ( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD ) // HID class command
                {
					switch( SetupReq )
					{
						case 0x01://GetReport
							 break;
						case 0x02://GetIdle
							 break;
						case 0x03://GetProtocol
							 break;
						case 0x09://SetReport
							 break;
						case 0x0A://SetIdle
							 break;
						case 0x0B://SetProtocol
							 break;
						default:
							 len = 0xFF; // command not supported
							 break;
					}
                }
                else
                { // Standard request
                    switch(SetupReq)                                        // Request code
                    {
                    case USB_GET_DESCRIPTOR:
                        switch(UsbSetupBuf->wValueH)
                        {
                        case 1:                                             // Device Descriptor
                            pDescr = DevDesc;                               // Premade buffer to be sent
                            len = sizeof(DevDesc);
                            break;
                        case 2:                                             // Configuration Descriptor
                            pDescr = CfgDesc;                               // Premade buffer to be sent
                            len = sizeof(CfgDesc);
                            break;
						case 3:												// String Descriptor
							len = UsbSetupBuf->wValueL;						// Index
							if (len < USB_STRINGDESC_COUNT) {
								pDescr = (uint8_t*)(StringDescs[len]);
								len = pDescr[0];
							} else {
								len = 0xFF;								// Not supported
							}
							break;
                        case 0x22:                                          //Report Descriptor
                            if (UsbSetupBuf->wIndexL < NUM_GAMEPADS)
                            {
                                pDescr = ControllerRepDesc[UsbSetupBuf->wIndexL];  // Premade buffer to be sent
                                len = REP_DESC_LEN;
                                if (UsbSetupBuf->wIndexL == (NUM_GAMEPADS-1)) {
                                  Ready = 1;
                                }
							}
							else {
                              	len = 0xff; // Normally wouldn't execute: Host should only ask for reports for 4 endpoints
                            }
                            break;
                        default:
                            len = 0xff; // Unsupported command or error
                            break;
                        }
                        if ( SetupLen > len )
                        {
                            SetupLen = len;    // Limit the total length
                        }
                        len = SetupLen >= 8 ? 8 : SetupLen; // Set current transmission length
                        memcpy(Ep0Buffer,pDescr,len);       // Set the data to upload
                        SetupLen -= len;					// SetupLen = remaining bytes to send
                        pDescr += len;						// Increment to get ready to send the next chunk (if exists)
                        break;
                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL;    // Set temporary USB device address
                        break;
                    case USB_GET_CONFIGURATION:
                        Ep0Buffer[0] = UsbConfig;
                        if ( SetupLen >= 1 )
                        {
                            len = 1;
                        }
                        break;
                    case USB_SET_CONFIGURATION:
                        UsbConfig = UsbSetupBuf->wValueL;
                        break;
                    case 0x0A:
                        break;
                    case USB_CLEAR_FEATURE:
                        if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP ) // Endpoint
                        {
                            switch( UsbSetupBuf->wIndexL )
							{
								case 0x84:
									UEP4_CTRL = UEP4_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
									break;
								case 0x83:
									UEP3_CTRL = UEP3_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
									break;
								case 0x82:
									UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
									break;
								case 0x81:
									UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
									break;
								case 0x01:
									UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
									break;
								default:
									len = 0xFF; // Unsupported endpoint
									break;
                            }
                        }
                        if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                        {
							break;
                        }
                        else
                        {
                            len = 0xFF; // Unsupported
                        }
                        break;
                    case USB_SET_FEATURE:
                        if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x00 ) // Setting up the device
                        {
                            if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
                            {
                                if( CfgDesc[ 7 ] & 0x20 )
                                {
                                    // Setting up wake up enable
                                }
                                else
                                {
                                    len = 0xFF; // Operation failed
                                }
                            }
                            else
                            {
                                len = 0xFF; // Operation failed
                            }
                        }
                        else if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x02 )  // Set endpoint
                        {
                            if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x00 )
                            {
                                switch( ( ( uint16_t )UsbSetupBuf->wIndexH << 8 ) | UsbSetupBuf->wIndexL )
                                {
									case 0x84:
										UEP4_CTRL = UEP4_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; // EP4 IN STALL
										break;
									case 0x83:
										UEP3_CTRL = UEP3_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; // EP3 IN STALL
										break;
									case 0x82:
										UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; // EP2 IN STALL
										break;
									case 0x81:
										UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; // EP1 IN STALL
										break;
									default:
										len = 0xFF; // Operation failed
										break;
                                }
                            }
                            else
                            {
                                len = 0xFF; // Operation failed
                            }
                        }
                        else
                        {
                            len = 0xFF; // Operation failed
                        }
                        break;
                    case USB_GET_STATUS:
                        Ep0Buffer[0] = 0x00;
                        Ep0Buffer[1] = 0x00;
                        if ( SetupLen >= 2 )
                        {
                            len = 2;
                        }
                        else
                        {
                            len = SetupLen;
                        }
                        break;
                    default:
                        len = 0xff; // Operation failed
                        break;
                    }
                }
            }
            else
            {
                len = 0xff; // Packet length error
            }
            if(len == 0xff)
            {
                SetupReq = 0xFF;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
            }
            else if(len) // Uploading data or status phase returns 0 length packets
            {
                UEP0_T_LEN = len;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; // Default is DATA1 + ACK
            }
            else
            {
				// Because it has not yet reached the status phase, it is preset to upload 0-length packets in advance
				// to prevent the host from entering the status phase too early.
                UEP0_T_LEN = 0;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; // Default is DATA1 + ACK
            }
            break;
        case UIS_TOKEN_IN | 0:                                               // Endpoint0 IN
            switch(SetupReq)
				{
				case USB_GET_DESCRIPTOR:
					len = SetupLen >= 8 ? 8 : SetupLen;                          // Current transmission length
					memcpy( Ep0Buffer, pDescr, len );                            // Data to send to host
					SetupLen -= len;	// SetupLen holds the remaining byte count
					pDescr += len;		// Increment pDescr to get ready for the next transaction
					UEP0_T_LEN = len;   // Tell the host that "len" bytes are available
					UEP0_CTRL ^= bUEP_T_TOG;                                     // Flip the sync flag
					break;
				case USB_SET_ADDRESS:
					USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
					UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
					break;
				default:
					UEP0_T_LEN = 0;  // Interruption of status phase completion or forced upload of
									 // 0-length data packets to end control transmission
					UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
					break;
            }
            break;
        case UIS_TOKEN_OUT | 0:  // endpoint0 OUT
            len = USB_RX_LEN;
            UEP0_CTRL ^= bUEP_R_TOG;                                     // Flip the sync flag
            break;
        default:
            break;
        }
        UIF_TRANSFER = 0;                                                 // Write 0 to clear interrupt
    }
    if(UIF_BUS_RST)                                                       // Device mode USB bus reset interrupt
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = UEP_T_RES_NAK;
        UEP2_CTRL = UEP_T_RES_NAK;
        UEP3_CTRL = UEP_T_RES_NAK;
        UEP4_CTRL = UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0;                                                 // Write 0 to clear interrupt
    }
    if (UIF_SUSPEND)                                                     // USB bus suspend / wake up completed
    {
        UIF_SUSPEND = 0;
        if ( USB_MIS_ST & bUMS_SUSPEND )                                 // ?
        {
        }
    }
    else {                  // Unexpected interrupt, should never happen
        USB_INT_FG = 0xFF;  // Clear all the interrupts and just blow out of the ISR.
    }
}

main()
{
    int i;
    CfgFsys( );
    mDelaymS(5);
    mInitSTDIO( );
    USBDeviceInit();

	// Put the I/O in a known state
    enableLiveAutodetect();
    CONTROLLER_INIT();

    for (i = 0; i < NUM_GAMEPADS; i++)
    {
        HIDCtrl[i][0] = i+1;
    }

    TMOD = 0x11;
    TH0 = (65536 - 2000)/256;
    TL0 = (65536 - 2000)%256;
    TR0 = 1;    // Start Timer 0
    ET0 = 1;    // Enable Timer 0 Interrupt
    EA  = 1;    // Global interrupt enable
	// Make sure we don't send any weird data on the IN endpoints...
    UEP1_T_LEN = 0;
    UEP2_T_LEN = 0;
    UEP3_T_LEN = 0;
    UEP4_T_LEN = 0;
    Ready = 0; // This gets set as soon as the last Report Descriptor is read.
    while(1)
    {
		if (millis-last > 40)
		{	// Toggle the LED to show that the thing is alive.
			LED1 = !LED1;
			last = millis;
		}
        GamepadGetLatest(); // Always get fresh data from the gamepads, because the fresher, the better!
        if(Ready)
        {
            HIDValueHandle(i); // Send it to the Host if the host wants it
        }
        i++;
        i %= NUM_GAMEPADS;
    }
}
