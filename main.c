typedef unsigned char                 *PUINT8;
typedef unsigned char volatile          UINT8V;

#include "ch554.h"
#include "debug.h"
#include "fournsnes.h"
#include <stdio.h>
#include <string.h>
#include <ch554_usb.h>

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
__xdata __at (0x0040) uint8_t  Ep4Buffer[16];
__xdata __at (0x0080) uint8_t  Ep1Buffer[16];
__xdata __at (0x00C0) uint8_t  Ep2Buffer[16];
__xdata __at (0x0100) uint8_t  Ep3Buffer[16];

uint8_t   SetupReq,SetupLen,Ready,Count,UsbConfig;
PUINT8  pDescr;
USB_SETUP_REQ   SetupReqBuf;

void	mTimer0Interrupt( void ) __interrupt (INT_NO_TMR0)
{
	TH0 = (65536 - 2000)/256;  // Reload
	TL0 = (65536 - 2000)%256;  // Reload
	millis++;
}

#define		BIT0		(0X01)
#define		BIT1		(0X02)
#define		BIT2		(0X04)
#define		BIT3		(0X08)
#define		BIT4		(0X10)
#define		BIT5		(0X20)
#define		BIT6		(0X40)
#define		BIT7		(0X80)

#define UsbSetupBuf     ((PUSB_SETUP_REQ)Ep0Buffer)

__code uint8_t DevDesc[18] = {18, // length (18)
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
8, 0,       // maximum packet size
10, // in ms
*/
#define CFG_DESC_LEN (9+((9+9+7)*4))
#define REP_DESC_LEN 45
#define CFG_INTERFACE_DESCR(a)  0x09,0x04,a,0x00,0x01,0x03,0x00,0x00,0x01
#define CFG_HID_DESCR  0x09,0x21,0x11,0x01,0x00,0x01,0x22,REP_DESC_LEN,0x00
#define CFG_EP_DESCR(a) 0x07,0x05,a,0x03,0x08,0x00,10

__code uint8_t CfgDesc[CFG_DESC_LEN] =
{
    9,          /* sizeof(usbDescriptorConfiguration): length of descriptor in bytes */
    0x02,    /* descriptor type */
    CFG_DESC_LEN,
    0,        /* total length of data returned (including inlined descriptors) */
    4,          /* number of interfaces in this configuration */
    1,          /* index of this configuration */
    0,          /* configuration name string index */
    0xA0, // USBATTR_BUSPOWER + USBATTR_REMOTEWAKE
    100/2,            /* max USB current in 2mA units */
/* interface descriptor follows inline: */

    CFG_INTERFACE_DESCR(0),
    CFG_HID_DESCR,
    CFG_EP_DESCR(0x81),
    CFG_INTERFACE_DESCR(1),
    CFG_HID_DESCR,
    CFG_EP_DESCR(0x82),
    CFG_INTERFACE_DESCR(2),
    CFG_HID_DESCR,
    CFG_EP_DESCR(0x83),
    CFG_INTERFACE_DESCR(3),
    CFG_HID_DESCR,
    CFG_EP_DESCR(0x84)
};

/*
0x05, 0x01, \			// USAGE_PAGE (Generic Desktop)
0x09, 0x04, \			// USAGE (Joystick)
0xa1, 0x01, \			//	COLLECTION (Application)
  0x09, 0x01, \			//		USAGE (Pointer)
  0xa1, 0x00, \			//		COLLECTION (Physical)
    0x85, a, \			  //			REPORT_ID (a)
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
  0xc0, \				//		END_COLLECTION
0xc0
*/
#define GAMEPAD_REPORT_DESCRIPTOR(a) 5,1,9,4,0xA1,1,9,1,0xA1,0,0x85,a, \
  9,0x30,9,0x31,0x15,0,0x26,0xFF,0,0x75,8,0x95,2,0x81,2,5,9,0x19,1,\
  0x29,NUM_BUTTONS,0x15,0,0x25,1,0x75,1,0x95,NUM_BUTTONS,0x81,2,0xC0,0xC0

__code uint8_t ControllerRepDesc[4][REP_DESC_LEN] = {
  {GAMEPAD_REPORT_DESCRIPTOR(1)},
  {GAMEPAD_REPORT_DESCRIPTOR(2)},
  {GAMEPAD_REPORT_DESCRIPTOR(3)},
  {GAMEPAD_REPORT_DESCRIPTOR(4)}
}; 			// END_COLLECTION

	// Language Descriptor
	__code uint8_t LangDesc[] = {
		4, 0x03,		// Length = 4 bytes, String Descriptor (0x03)
		0x09, 0x04	// 0x0409 English - United States
	};

	// String Descriptors:
	__code uint8_t DevName1[] = {
		34, 0x03, 	// Length = 34 bytes, String Descriptor (0x03)
		'N', 0, 'E', 0, 'S', 0, '/', 0, 'S', 0, 'N', 0, 'E', 0, 'S', 0,
		' ', 0, 'G', 0, 'a', 0, 'm', 0, 'e', 0, 'p', 0, 'a', 0, 'd', 0
	};

	__code uint8_t ProductName[] = {
		20, 0x03, 	// Length = 20 bytes, String Descriptor (0x03)
		'4', 0, 'N', 0, 'E', 0, 'S', 0, '4', 0, 'S', 0, 'N', 0, 'E', 0, 'S', 0
	};

	__code uint8_t ManuName[] = {
		16, 0x03, 	// Length = 30 bytes, String Descriptor (0x03)
		'R', 0, 'a', 0, 'p', 0, 'h', 0, 'n', 0, 'e', 0, 't', 0
	};

#define USB_STRINGDESC_COUNT 4
	__code uint8_t* StringDescs[USB_STRINGDESC_COUNT] = {
		LangDesc,			// 0 (If you want to support string descriptors, you must have this!)
		DevName1,			// 1
		ProductName,	// 2
		ManuName			// 3
	};

#if NUM_BUTTONS > 8
#define GAMEPAD_XMIT_DATA_LEN 5
#else
#define GAMEPAD_XMIT_DATA_LEN 4
#endif

uint8_t HIDCtrl1[5] = {0x1,0x80,0x80,0x0,0x0};
uint8_t HIDCtrl2[5] = {0x2,0x80,0x80,0x0,0x0};
uint8_t HIDCtrl3[5] = {0x3,0x80,0x80,0x0,0x0};
uint8_t HIDCtrl4[5] = {0x4,0x80,0x80,0x0,0x0};

void CH554SoftReset( )
{
  SAFE_MOD = 0x55;
  SAFE_MOD = 0xAA;
  GLOBAL_CFG	|=bSW_RESET;
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
    USB_CTRL = 0x00;                                                           // ���趨USB�豸ģʽ
    UEP2_DMA = (uint16_t)Ep2Buffer; // Set endpoint 2's DMA buffer to Ep2Buffer
    UEP3_DMA = (uint16_t)Ep3Buffer; // Set endpoint 3's DMA buffer to Ep3Buffer
    UEP2_3_MOD = bUEP3_TX_EN | bUEP2_TX_EN; // Set both endpoints 2 and 3 as IN endpoints
    UEP2_CTRL = UEP_T_RES_NAK;                                 //�˵�2�Զ���תͬ����־λ��IN���񷵻�NAK
    UEP3_CTRL = UEP_T_RES_NAK;                                 //�˵�2�Զ���תͬ����־λ��IN���񷵻�NAK
    UEP0_DMA = (uint16_t)Ep0Buffer;  // Set endpoint 0's DMA buffer to Ep0Buffer (and Ep4Buffer gets set to Ep0Buffer+0x40)
    UEP1_DMA = (uint16_t)Ep1Buffer;  // Set endpoint 1's DMA buffer to Ep3Buffer
    UEP4_1_MOD = bUEP1_TX_EN | bUEP4_TX_EN; // Set both endpoints 1 and 4 as IN endpoints
    UEP1_CTRL = UEP_T_RES_NAK;                                 //�˵�1�Զ���תͬ����־λ��IN���񷵻�NAK
    UEP4_CTRL = UEP_T_RES_NAK;                                 //�˵�1�Զ���תͬ����־λ��IN���񷵻�NAK
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;                                 //OUT���񷵻�ACK��IN���񷵻�NAK

	  USB_DEV_AD = 0x00;
	  UDEV_CTRL = bUD_PD_DIS;                                                    // ��ֹDP/DM��������
	  USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;                      // ����USB�豸��DMA�����ж��ڼ��жϱ�־δ����ǰ�Զ�����NAK
	  UDEV_CTRL |= bUD_PORT_EN;                                                  // ����USB�˿�
	  USB_INT_FG = 0xFF;                                                         // ���жϱ�־
	  USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
	  IE_USB = 1;
}

void Enp1IntIn( )
{
    memcpy( Ep1Buffer, HIDCtrl1, GAMEPAD_XMIT_DATA_LEN);
    UEP1_T_LEN = GAMEPAD_XMIT_DATA_LEN;
    UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
}

void Enp2IntIn( )
{
    memcpy( Ep2Buffer, HIDCtrl2, GAMEPAD_XMIT_DATA_LEN);
    UEP2_T_LEN = GAMEPAD_XMIT_DATA_LEN;
    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
}

void Enp3IntIn( )
{
    memcpy( Ep3Buffer, HIDCtrl3, GAMEPAD_XMIT_DATA_LEN);
    UEP3_T_LEN = GAMEPAD_XMIT_DATA_LEN;
    UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
}

void Enp4IntIn( )
{
    memcpy( Ep4Buffer, HIDCtrl4, GAMEPAD_XMIT_DATA_LEN);
    UEP4_T_LEN = GAMEPAD_XMIT_DATA_LEN;
    UEP4_CTRL = UEP4_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
}

void HIDValueHandle()
{
  Enp1IntIn();
  Enp2IntIn();
  Enp3IntIn();
  Enp4IntIn();
}

void GamepadGetLatest()
{
  fournsnesUpdate();
  fournsnesBuildReport(HIDCtrl1, 1);
  fournsnesBuildReport(HIDCtrl2, 2);
  fournsnesBuildReport(HIDCtrl3, 3);
  fournsnesBuildReport(HIDCtrl4, 4);
}

#define SETUP_NEXT_EP_VALUE 0
void DeviceInterrupt(void) __interrupt (INT_NO_USB)
{
    uint8_t len = 0;
    if(UIF_TRANSFER)                                                            //USB�������ɱ�־
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
        case UIS_TOKEN_IN | 4:                                                  //endpoint 4# �ж϶˵��ϴ�
            UEP4_T_LEN = 0;
            UEP4_CTRL = UEP4_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;
						UEP4_CTRL ^= bUEP_T_TOG;
            #if SETUP_NEXT_EP_VALUE
						memcpy( Ep4Buffer, HIDCtrl4, 5);
						UEP4_T_LEN = 5;
						UEP4_CTRL = UEP4_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
            #endif
            break;
        case UIS_TOKEN_IN | 3:                                                  //endpoint 3# �ж϶˵��ϴ�
          UEP3_T_LEN = 0;
          UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;
					UEP3_CTRL ^= bUEP_T_TOG;
          #if SETUP_NEXT_EP_VALUE
					memcpy( Ep3Buffer, HIDCtrl3, 5);
					UEP3_T_LEN = 5;
					UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
          #endif
            break;
        case UIS_TOKEN_IN | 2:                                                  //endpoint 2# �ж϶˵��ϴ�
          UEP2_T_LEN = 0;
          UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;
					UEP2_CTRL ^= bUEP_T_TOG;
          #if SETUP_NEXT_EP_VALUE
					memcpy( Ep2Buffer, HIDCtrl2, 5);
					UEP2_T_LEN = 5;
					UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
          #endif
            break;
        case UIS_TOKEN_IN | 1:                                                  //endpoint 1# �ж϶˵��ϴ�
          UEP1_T_LEN = 0;
          UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;
					UEP1_CTRL ^= bUEP_T_TOG;
          #if SETUP_NEXT_EP_VALUE
					memcpy( Ep1Buffer, HIDCtrl1, 5);
					UEP1_T_LEN = 5;
					UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
          #endif
            break;
        case UIS_TOKEN_SETUP | 0:                                                //SETUP����
            len = USB_RX_LEN;
            if(len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = UsbSetupBuf->wLengthL;
                if(UsbSetupBuf->wLengthH || SetupLen > 0x7F )
                {
                    SetupLen = 0x7F;    // �����ܳ���
                }
                len = 0;                                                        // Ĭ��Ϊ�ɹ������ϴ�0����
                SetupReq = UsbSetupBuf->bRequest;
                if ( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )/* HID������ */
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
												 len = 0xFF;  								 					            /*���֧��*/
												 break;
								  }
                }
                else
                {//��׼����
                    switch(SetupReq)                                        //������
                    {
                    case USB_GET_DESCRIPTOR:
                        switch(UsbSetupBuf->wValueH)
                        {
                        case 1:                                             //�豸������
                            pDescr = DevDesc;                               //���豸�������͵�Ҫ���͵Ļ�����
                            len = sizeof(DevDesc);
                            break;
                        case 2:                                             //����������
                            pDescr = CfgDesc;                               //���豸�������͵�Ҫ���͵Ļ�����
                            len = sizeof(CfgDesc);
                            break;
												case 3:												// String Descriptor
													len = UsbSetupBuf->wValueL;	// Index
													if (len < USB_STRINGDESC_COUNT) {
														pDescr = (uint8_t*)(StringDescs[len]);
														len = pDescr[0];
													} else {
														len = 0xFF;								// Not supported
													}
													break;
                        case 0x22:                                          //Report Descriptor
                            if (UsbSetupBuf->wIndexL < 4)
                            {
                                pDescr = ControllerRepDesc[UsbSetupBuf->wIndexL];                      //Data ready to upload
                                len = REP_DESC_LEN;
                                if (UsbSetupBuf->wIndexL == 3) {
                                  Ready = 1;
                                }
														}
														else {
                              	len = 0xff; // Normally wouldn't execute
                            }
                            break;
                        default:
                            len = 0xff;                                     //��֧�ֵ��������߳���
                            break;
                        }
                        if ( SetupLen > len )
                        {
                            SetupLen = len;    //�����ܳ���
                        }
                        len = SetupLen >= 8 ? 8 : SetupLen;                  //���δ��䳤��
                        memcpy(Ep0Buffer,pDescr,len);                        //�����ϴ�����
                        SetupLen -= len;
                        pDescr += len;
                        break;
                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL;                     //�ݴ�USB�豸��ַ
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
                    case USB_CLEAR_FEATURE:                                            //Clear Feature
                        if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )// �˵�
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
                                len = 0xFF;                                            // ��֧�ֵĶ˵�
                                break;
                            }
                        }
                        if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )// �豸
                        {
													break;
                        }
                        else
                        {
                            len = 0xFF;                                                // ���Ƕ˵㲻֧��
                        }
                        break;
                    case USB_SET_FEATURE:                                              /* Set Feature */
                        if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x00 )             /* �����豸 */
                        {
                            if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
                            {
                                if( CfgDesc[ 7 ] & 0x20 )
                                {
                                    /* ���û���ʹ�ܱ�־ */
                                }
                                else
                                {
                                    len = 0xFF;                                        /* ����ʧ�� */
                                }
                            }
                            else
                            {
                                len = 0xFF;                                            /* ����ʧ�� */
                            }
                        }
                        else if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x02 )        /* ���ö˵� */
                        {
                            if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x00 )
                            {
                                switch( ( ( uint16_t )UsbSetupBuf->wIndexH << 8 ) | UsbSetupBuf->wIndexL )
                                {
                                case 0x84:
                                    UEP4_CTRL = UEP4_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* ���ö˵�4 IN STALL */
                                    break;
                                case 0x83:
                                    UEP3_CTRL = UEP3_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* ���ö˵�3 IN STALL */
                                    break;
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* ���ö˵�2 IN STALL */
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* ���ö˵�1 IN STALL */
                                    break;
                                default:
                                    len = 0xFF;                               //����ʧ��
                                    break;
                                }
                            }
                            else
                            {
                                len = 0xFF;                                   //����ʧ��
                            }
                        }
                        else
                        {
                            len = 0xFF;                                      //����ʧ��
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
                        len = 0xff;                                           //����ʧ��
                        break;
                    }
                }
            }
            else
            {
                len = 0xff;                                                   //�����ȴ���
            }
            if(len == 0xff)
            {
                SetupReq = 0xFF;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
            }
            else if(len)                                                //�ϴ����ݻ���״̬�׶η���0���Ȱ�
            {
                UEP0_T_LEN = len;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//Ĭ�����ݰ���DATA1������Ӧ��ACK
            }
            else
            {
                UEP0_T_LEN = 0;  //��Ȼ��δ��״̬�׶Σ�������ǰԤ���ϴ�0�������ݰ��Է�������ǰ����״̬�׶�
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//Ĭ�����ݰ���DATA1,����Ӧ��ACK
            }
            break;
        case UIS_TOKEN_IN | 0:                                               //endpoint0 IN
            switch(SetupReq)
            {
            case USB_GET_DESCRIPTOR:
                len = SetupLen >= 8 ? 8 : SetupLen;                          //���δ��䳤��
                memcpy( Ep0Buffer, pDescr, len );                            //�����ϴ�����
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG;                                     //ͬ����־λ��ת
                break;
            case USB_SET_ADDRESS:
                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            default:
                UEP0_T_LEN = 0;                                              //״̬�׶������жϻ�����ǿ���ϴ�0�������ݰ��������ƴ���
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            }
            break;
        case UIS_TOKEN_OUT | 0:  // endpoint0 OUT
            len = USB_RX_LEN;
            UEP0_CTRL ^= bUEP_R_TOG;                                     //ͬ����־λ��ת
            break;
        default:
            break;
        }
        UIF_TRANSFER = 0;                                                 //д0�����ж�
    }
    if(UIF_BUS_RST)                                                       //�豸ģʽUSB���߸�λ�ж�
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = UEP_T_RES_NAK;
        UEP2_CTRL = UEP_T_RES_NAK;
        UEP3_CTRL = UEP_T_RES_NAK;
        UEP4_CTRL = UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0;                                                 //?
    }
    if (UIF_SUSPEND)                                                     //USB���߹���/��������
    {
        UIF_SUSPEND = 0;
        if ( USB_MIS_ST & bUMS_SUSPEND )                                 //?
        {
        }
    }
    else {                                                               //�������ж�,�����ܷ���������
        USB_INT_FG = 0xFF;
    }
}

main()
{
    CfgFsys( );
    mDelaymS(5);
    mInitSTDIO( );
    USBDeviceInit();

    P3_MOD_OC = P3_MOD_OC | (1<<LED_PIN1) | (1<<MULTITAP_PIN) | (1<<LATCH_PIN) | (1<<CLOCK_PIN); // 1 = push/pull, 0 = open-drain
    P3_DIR_PU = P3_DIR_PU | (1<<LED_PIN1) | (1<<MULTITAP_PIN) | (1<<LATCH_PIN) | (1<<CLOCK_PIN); // 1 = output, 0 = input (if push-pull)
    P1_MOD_OC = P1_MOD_OC &= ~((1<<DATA0_PIN) | (1<<DATA1_PIN) | (1<<DATA2_PIN) | (1<<DATA3_PIN)); // Make them all open drain
    P1_DIR_PU = P1_DIR_PU | (1<<DATA0_PIN) | (1<<DATA1_PIN) | (1<<DATA2_PIN) | (1<<DATA3_PIN); // pull-up enable (if open drain)

    fournsnesInit();

    TMOD = 0x11;
    TH0 = (65536 - 2000)/256;
    TL0 = (65536 - 2000)%256;
    TR0 = 1;    // Start Timer 0
    ET0 = 1;    // Enable Timer 0 Interrupt
    EA  = 1;    // Global interrupt enable
    UEP1_T_LEN = 0;
    UEP2_T_LEN = 0;
    UEP3_T_LEN = 0;
    UEP4_T_LEN = 0;
    Ready = 0;
    while(1)
    {
				if (millis-last>40){
	          LED1 = !LED1;
	          last=millis;
				}
        GamepadGetLatest();
        if(Ready)
        {
            HIDValueHandle();
        }
    }
}
