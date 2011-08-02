/***********************************************************************************/
/*  Copyright (c) 2002-2009, Silicon Image, Inc.  All rights reserved.             */
/*  No part of this work may be reproduced, modified, distributed, transmitted,    */
/*  transcribed, or translated into any language or computer format, in any form   */
/*  or by any means without written permission of: Silicon Image, Inc.,            */
/*  1060 East Arques Avenue, Sunnyvale, California 94085                           */
/***********************************************************************************/

// Standard C Library

// Needs to be rearranged and minimized
#include <linux/time.h>
#include <linux/gpio.h>
#include <asm/io.h>
#include <plat/imapx.h>
#include <linux/delay.h>
#include "SIIdefs.h"
#include "SIITypeDefs.h"
//#include "AMF_Lib.h"
#include "SIIConstants.h"
#include "SIIMacros.h"
#include "SIITPI_Regs.h"
#include "SIITPI_Access.h"
#include "SIITPI.h"
#include "SIIAV_Config.h"
#include "Externals.h"


// May be empty depending on project definitions
#include "SIIedid.h"
#include "SIIHDCP.h"


static void TxHW_Reset(void);
static bool StartTPI(void);

static void ForceUsbIdSwitchOpen (void);
static void ReleaseUsbIdSwitchOpen (void);

static bool DisableInterrupts(unsigned char);

static bool TestHPD_Connected(void);

static bool SetPreliminaryInfoFrames(void);



//static void OnHdmiCableConnected(void);
//static void OnHdmiCableDisconnected(void);

static void OnDownstreamRxPoweredDown(void);
static void OnDownstreamRxPoweredUp(void);

void TxPowerState(unsigned char powerState);

#define T_EN_TPI       	10
#define T_HPD_DELAY    	10



bool tmdsPoweredUp;
static bool hdmiCableConnected;
static bool dsRxPoweredUp;
static const char TPI_FW_VERSION[] = {'0', '.', '2', '8'};

//zhy + Begin
void VideoModeSetbymain(void);
void AudioModeSetbymain(void);
//zhy + End

//////////////////////////////////////////////////////////////////////////////
////
//// FUNCTION      :  DelayMS(unsigned int M_Sec)
////
//// PURPOSE       :  Introduce a busy-wait delay equal, in milliseconds, to the
////                  input parameter.
////
//// INPUT PARAMS  :  Length of required delay in milliseconds (max. 65535 ms)
////
//// OUTPUT PARAMS :  None
////
//// GLOBALS USED  :  None
////
//// RETURNS       :  void
////
////////////////////////////////////////////////////////////////////////////////
//
void DelayMS(int M_Sec)
{
	unsigned int i;
	for(i=0;i<M_Sec;i++)
		udelay(1000);
		//msleep(M_Sec);
}


//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION      : TPI_Init()
//
// PURPOSE       : TPI initialization: HW Reset, Interrupt enable.
//
// INPUT PARAMS  : None
//
// OUTPUT PARAMS : void
//
// GLOBALS USED  : None
//
// RETURNS      :   TRUE
//
//////////////////////////////////////////////////////////////////////////////
bool TPI_Init(void)
{
	int i = 0;
	TPI_TRACE_PRINT((">>TPI_Init()\n"));

	printk("TPI Firmware Version ");
	for (i=0; i < (int)sizeof(TPI_FW_VERSION); i++) {
		printk("%c", TPI_FW_VERSION[i]);
	}
	printk("\n");

	tmdsPoweredUp = FALSE;
	hdmiCableConnected = FALSE;
	dsRxPoweredUp = FALSE;

#ifdef DEV_SUPPORT_EDID
	edidDataValid = FALSE;							// Move this into EDID_Init();
#endif

	TxHW_Reset();									// Toggle TX reset pin

	if (StartTPI())									// Enable HW TPI mode, check device ID
	{
#ifdef DEV_SUPPORT_HDCP
		HDCP_Init();
#endif

		return TRUE;
	}

	return FALSE;
}




//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION     :   HotPlugService()
//
// PURPOSE      :   Implement Hot Plug Service Loop activities
//
// INPUT PARAMS :   None
//
// OUTPUT PARAMS:   void
//
// GLOBALS USED :   LinkProtectionLevel
//
// RETURNS      :   An error code that indicates success or cause of failure
//
//////////////////////////////////////////////////////////////////////////////

void HotPlugService (void)
{
	TPI_TRACE_PRINT((">>HotPlugService()\n"));

	DisableInterrupts(0xFF);

	if(IsHDMI_Sink())	//Set InfoFrames only if HDMI output mode
	{
		dvi_mode = FALSE;
	}
	else
	{
		dvi_mode = TRUE;
	}

#ifndef RX_ONBOARD
	vid_mode = EDID_Data.VideoDescriptor[0];					// use 1st mode supported by sink
#endif

	//zhy Modify for hotplug Begin
	VideoModeSetbymain();	

	if (IsHDMI_Sink())											// Set audio only if sink is HDMI compatible
	{
		AudioModeSetbymain();
	}
	else
	{
		SetAudioMute(AUDIO_MUTE_MUTED);
	}

#if 0
	InitVideo(vid_mode, X1, AFTER_INIT, NO_3D_SUPPORT);			// Set PLL Multiplier to x1 upon power up

	TxPowerState(TX_POWER_STATE_D0);

	if (IsHDMI_Sink())											// Set InfoFrames only if HDMI output mode
	{
		SetPreliminaryInfoFrames();
		SetBasicAudio();										// set audio interface to basic audio (an external command is needed to set to any other mode
		dvi_mode = FALSE;
	}
	else
	{
		SetAudioMute(AUDIO_MUTE_MUTED);
		dvi_mode = TRUE;
	}

#ifdef DEV_SUPPORT_EHDMI
	//	EHDMI_ARC_Common_Enable();
	EHDMI_ARC_Common_With_HEC_Enable();
#endif

	EnableInterrupts(HOT_PLUG_EVENT | RX_SENSE_EVENT | AUDIO_ERROR_EVENT | SECURITY_CHANGE_EVENT | V_READY_EVENT | HDCP_CHANGE_EVENT);

	// This check needs to be changed to if HDCP is required by the content... once support has been added by RX-side library.
	if (HDCP_TxSupports == TRUE)
	{
#ifdef USE_BLACK_MODE
		SetInputColorSpace (INPUT_COLOR_SPACE_BLACK_MODE);
		TPI_DEBUG_PRINT (("TMDS -> Enabled\n"));
		ReadModifyWriteTPI(TPI_SYSTEM_CONTROL_DATA_REG, LINK_INTEGRITY_MODE_MASK | TMDS_OUTPUT_CONTROL_MASK, LINK_INTEGRITY_DYNAMIC | TMDS_OUTPUT_CONTROL_ACTIVE);
#else	// AV MUTE
		TPI_DEBUG_PRINT (("TMDS -> Enabled (Video Muted)\n"));
		ReadModifyWriteTPI(TPI_SYSTEM_CONTROL_DATA_REG, LINK_INTEGRITY_MODE_MASK | TMDS_OUTPUT_CONTROL_MASK | AV_MUTE_MASK, LINK_INTEGRITY_DYNAMIC | TMDS_OUTPUT_CONTROL_ACTIVE | AV_MUTE_MUTED);
#endif
		tmdsPoweredUp = TRUE;
	}
	else
	{
		EnableTMDS();
	}
#endif
	EnableInterrupts(HOT_PLUG_EVENT | RX_SENSE_EVENT | AUDIO_ERROR_EVENT | SECURITY_CHANGE_EVENT | V_READY_EVENT | HDCP_CHANGE_EVENT);
	//zhy Modify for hotplug End
}


//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION      : HW_Reset()
//
// PURPOSE       : Send a
//
// INPUT PARAMS  : None
//
// OUTPUT PARAMS : void
//
// GLOBALS USED  : None
//
// RETURNS       : Void
//
//////////////////////////////////////////////////////////////////////////////

static void TxHW_Reset(void)
{
	unsigned int reg_temp;

	reg_temp = __imapx_name_to_gpio(CONFIG_HDMI_SI9022_RESET);

	if(reg_temp == IMAPX_GPIO_ERROR)
	{
		printk(KERN_ERR "SI9022 get reset pin failed.\n");
		return -ENOTTY;
	}

	imapx_gpio_setcfg(reg_temp, IG_OUTPUT, IG_NORMAL);

	TPI_TRACE_PRINT((">>TxHW_Reset()\n"));

//	CYC_MICRO_RESET = LOW;
	imapx_gpio_setpin(reg_temp, 0, IG_NORMAL);

	DelayMS(TX_HW_RESET_PERIOD);
//	CYC_MICRO_RESET = HIGH;
	imapx_gpio_setpin(reg_temp, 1, IG_NORMAL);


	TXHAL_InitPostReset();	//backdoor register access, enable source termination

	// Does this need to be done for every chip? Should it be moved into TXHAL_InitPostReset() for each applicable device?
	// HW Debouce of what?
//	I2C_WriteByte(0x72, 0x7C, 0x14);				// HW debounce to 64ms (0x14)
	I2C_WriteByte(0x39, 0x7C, 0x14);				// HW debounce to 64ms (0x14)
}


//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION      : StartTPI()
//
// PURPOSE       : Start HW TPI mode by writing 0x00 to TPI address 0xC7. This
//                 will take the Tx out of power down mode.
//
// INPUT PARAMS  : None
//
// OUTPUT PARAMS : void
//
// GLOBALS USED  : None
//
// RETURNS       : TRUE if HW TPI started successfully. FALSE if failed to.
//
//////////////////////////////////////////////////////////////////////////////

static bool StartTPI(void)
{
	unsigned char devID = 0x00;
	unsigned int wID = 0x0000;

	TPI_TRACE_PRINT((">>StartTPI()\n"));

	WriteByteTPI(TPI_ENABLE, 0x00);            // Write "0" to 72:C7 to start HW TPI mode
	DelayMS(100);

	devID = ReadIndexedRegister(INDEXED_PAGE_0, 0x03);
	wID = devID;
	wID <<= 8;
	devID = ReadIndexedRegister(INDEXED_PAGE_0, 0x02);
	wID |= devID;

	devID = ReadByteTPI(TPI_DEVICE_ID);

	printk ("0x%04X\n", (int) wID);

	if (devID == SiI_DEVICE_ID) {
		//printk (SiI_DEVICE_STRING);
		return TRUE;
	}

	printk ("Unsupported TX\n");
	return FALSE;
}


//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION      :  EnableInterrupts()
//
// PURPOSE       :  Enable the interrupts specified in the input parameter
//
// INPUT PARAMS  :  A bit pattern with "1" for each interrupt that needs to be
//                  set in the Interrupt Enable Register (TPI offset 0x3C)
//
// OUTPUT PARAMS :  void
//
// GLOBALS USED  :  None
//
// RETURNS       :  TRUE
//
//////////////////////////////////////////////////////////////////////////////
bool EnableInterrupts(unsigned char Interrupt_Pattern)
{
	TPI_TRACE_PRINT((">>EnableInterrupts()\n"));
	ReadSetWriteTPI(TPI_INTERRUPT_ENABLE_REG, Interrupt_Pattern);

	return TRUE;
}


//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION      :  DisableInterrupts()
//
// PURPOSE       :  Enable the interrupts specified in the input parameter
//
// INPUT PARAMS  :  A bit pattern with "1" for each interrupt that needs to be
//                  cleared in the Interrupt Enable Register (TPI offset 0x3C)
//
// OUTPUT PARAMS :  void
//
// GLOBALS USED  :  None
//
// RETURNS       :  TRUE
//
//////////////////////////////////////////////////////////////////////////////
static bool DisableInterrupts(unsigned char Interrupt_Pattern)
{
	TPI_TRACE_PRINT((">>DisableInterrupts()\n"));
	ReadClearWriteTPI(TPI_INTERRUPT_ENABLE_REG, Interrupt_Pattern);

	return TRUE;
}





//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION      : SetPreliminaryInfoFrames()
//
// PURPOSE       : Set InfoFrames for default (initial) setup only
//
// INPUT PARAMS  : VIC, output mode,
//
// OUTPUT PARAMS : void
//
// GLOBALS USED  : None
//
// RETURNS       : TRUE
//
//////////////////////////////////////////////////////////////////////////////
static bool SetPreliminaryInfoFrames()
{
	unsigned char i;
	API_Cmd Command;        // to allow using function SetAVI_InfoFrames()

	TPI_TRACE_PRINT((">>SetPreliminaryInfoFrames()\n"));

	for (i = 0; i < MAX_COMMAND_ARGUMENTS; i++)
		Command.Arg[i] = 0;

	Command.CommandLength = 0;      // fixes SetAVI_InfoFrames() from going into an infinite loop

	Command.Arg[0] = vid_mode;

#ifdef DEV_SUPPORT_EDID
	if (EDID_Data.YCbCr_4_4_4)
	{
		Command.Arg[3] = 0x01;
	}
	else
	{
		if (EDID_Data.YCbCr_4_2_2)
		{
			Command.Arg[3] = 0x02;
		}
	}
#else
	Command.Arg[3] = 0x00;
#endif

	SetAVI_InfoFrames(Command);
	return TRUE;
}


void TxPowerState(unsigned char powerState) {

	TPI_DEBUG_PRINT(("TX Power State D%d\n", (int)powerState));
	/* Eric add infoTM*/
	unsigned char InterruptStatusImage;
	/* Eric add infoTM*/

	/* Eric add infoTM*/
	if(powerState == TX_POWER_STATE_D3)		//defaust to D3 Cold mode
	{
		InterruptStatusImage = ReadByteTPI(TPI_INTERRUPT_STATUS_REG);    // Read Interrupt status register
		if((InterruptStatusImage & HOT_PLUG_STATE)  == 0x00)			//HPD low
		{
			ReadSetWriteTPI(TPI_INTERRUPT_STATUS_REG, 0xF9); //Clear all pending interrupts
			TxHW_Reset();
			StartTPI();
			ReadSetWriteTPI(TPI_INTERRUPT_ENABLE_REG, HOT_PLUG_EVENT); //enable hotplug interrupt
			ReadSetWriteTPI(TPI_INTERRUPT_STATUS_REG, 0xF9); //Clear all pending interrupts again
			if((InterruptStatusImage & HOT_PLUG_STATE)  == 0x00)			//HPD low
			{
				ReadModifyWriteTPI(TPI_DEVICE_POWER_STATE_CTRL_REG, 0x04, 0x04);	//set Cold bit
//				ReadModifyWriteTPI(TPI_DEVICE_POWER_STATE_CTRL_REG, TX_POWER_STATE_MASK, powerState);	//set to D3 mode
				TPI_DEBUG_PRINT(("Entered D3 cold mode\n"));

			}
			else{
				TPI_DEBUG_PRINT(("HPD high, should not enter D3 cold mode ,D3 hot only\n"));
				powerState = TX_POWER_STATE_D2;
//				return;
			}
		}
		else
		{
			TPI_DEBUG_PRINT(("HPD high, should not enter D3 cold mode,D3 hot only\n"));
			powerState = TX_POWER_STATE_D2;
//			return;
		}

	}

	/* end Eric add infoTM*/
	ReadModifyWriteTPI(TPI_DEVICE_POWER_STATE_CTRL_REG, TX_POWER_STATE_MASK, powerState);
}


void EnableTMDS (void) {

	TPI_DEBUG_PRINT(("TMDS -> Enabled\n"));
	ReadModifyWriteTPI(TPI_SYSTEM_CONTROL_DATA_REG, TMDS_OUTPUT_CONTROL_MASK, TMDS_OUTPUT_CONTROL_ACTIVE);
	tmdsPoweredUp = TRUE;
}


void DisableTMDS (void) {

	TPI_DEBUG_PRINT(("TMDS -> Disabled\n"));
	//ReadModifyWriteTPI(TPI_SYSTEM_CONTROL_DATA_REG, TMDS_OUTPUT_CONTROL_MASK, TMDS_OUTPUT_CONTROL_POWER_DOWN);
	ReadModifyWriteTPI(TPI_SYSTEM_CONTROL_DATA_REG, TMDS_OUTPUT_CONTROL_MASK | AV_MUTE_MASK, TMDS_OUTPUT_CONTROL_POWER_DOWN | AV_MUTE_MUTED);
	tmdsPoweredUp = FALSE;
}

#ifdef DEV_SUPPORT_HDCP
void RestartHDCP (void)
{
	TPI_DEBUG_PRINT (("HDCP -> Restart\n"));

	DisableTMDS();
	HDCP_Off();
	EnableTMDS();
}
#endif

void SetAudioMute (unsigned char audioMute)
{
	ReadModifyWriteTPI(TPI_AUDIO_INTERFACE_REG, AUDIO_MUTE_MASK, audioMute);
}


//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION     :   TPI_Poll ()
//
// PURPOSE      :   Poll Interrupt Status register for new interrupts
//
// INPUT PARAMS :   None
//
// OUTPUT PARAMS:   None
//
// GLOBALS USED :   LinkProtectionLevel
//
// RETURNS      :   None
//
//////////////////////////////////////////////////////////////////////////////

void TPI_Poll (void)
{
	unsigned char InterruptStatusImage;
	unsigned int timing = 0;

	InterruptStatusImage = ReadByteTPI(TPI_INTERRUPT_STATUS_REG);

	if (InterruptStatusImage & HOT_PLUG_EVENT)
	{
		TPI_DEBUG_PRINT (("HPD  -> "));

		ReadSetWriteTPI(TPI_INTERRUPT_ENABLE_REG, HOT_PLUG_EVENT);  // Enable HPD interrupt bit

		// Repeat this loop while cable is bouncing:
		do
		{
			WriteByteTPI(TPI_INTERRUPT_STATUS_REG, HOT_PLUG_EVENT);
			DelayMS(T_HPD_DELAY); // Delay for metastability protection and to help filter out connection bouncing
			InterruptStatusImage = ReadByteTPI(TPI_INTERRUPT_STATUS_REG);    // Read Interrupt status register
		} while (InterruptStatusImage & HOT_PLUG_EVENT);              // loop as long as HP interrupts recur

		if (((InterruptStatusImage & HOT_PLUG_STATE) >> 2) != hdmiCableConnected)
		{
			if (hdmiCableConnected == TRUE)
			{
				OnHdmiCableDisconnected();
			}

			else
			{
				OnHdmiCableConnected(timing);
				ReadModifyWriteIndexedRegister(INDEXED_PAGE_0, 0x0A, 0x08, 0x08);
			}

			if (hdmiCableConnected == FALSE)
			{
				return;
			}
		}
	}





#if !defined SiI9232_OR_SiI9236
	// Check rx power
	if (((InterruptStatusImage & RX_SENSE_STATE) >> 3) != dsRxPoweredUp)
	{
		if (hdmiCableConnected == TRUE)
		{
			if (dsRxPoweredUp == TRUE)
			{
				OnDownstreamRxPoweredDown();
			}

			else
			{
				OnDownstreamRxPoweredUp();
			}
		}

		ClearInterrupt(RX_SENSE_EVENT);
	}
#endif

	// Check if Audio Error event has occurred:
	if (InterruptStatusImage & AUDIO_ERROR_EVENT)
	{
		//TPI_DEBUG_PRINT (("TP -> Audio Error Event\n"));
		//  The hardware handles the event without need for host intervention (PR, p. 31)
		ClearInterrupt(AUDIO_ERROR_EVENT);
	}

#ifdef DEV_SUPPORT_HDCP
	if ((hdmiCableConnected == TRUE) && (dsRxPoweredUp == TRUE))
	{
		HDCP_CheckStatus(InterruptStatusImage);
	}
#endif

#ifdef RX_ONBOARD
	if ((tmdsPoweredUp == TRUE) && (pvid_mode != vid_mode))
	{
		TPI_TRACE_PRINT(("TP -> vid_mode...\n"));
		DisableTMDS();
		HotPlugService();
		pvid_mode = vid_mode;
	}
#endif



}




void OnHdmiCableConnected(LCD_TIMING timing)
{
	TPI_DEBUG_PRINT (("HDMI Connected ("));

	switch(timing)
	{
		case HDMI_1080P:
			VideoMode[0] = 16;
			break;
		case HDMI_720P:
			VideoMode[0] = 4;
			break;
		case HDMI_480P_16_9:
			VideoMode[0] = 3;
			break;
		case HDMI_480P_4_3:
			VideoMode[0] = 2;
			break;
		case HDMI_576P_16_9:
			VideoMode[0] = 18;
			break;
		case HDMI_576P_4_3:
			VideoMode[0] = 17;
			break;
		case HDMI_640_480:
			VideoMode[0] = 1;
			break;
		case HDMI_800_600:
			VideoMode[0] = 65;
			break;
		case HDMI_1024_768:
			VideoMode[0] = 69;
			break;
		default:
			break;

	}

	hdmiCableConnected = TRUE;

#ifdef DEV_SUPPORT_HDCP
	WriteIndexedRegister(INDEXED_PAGE_0, 0xCE, 0x00); // Clear BStatus
	WriteIndexedRegister(INDEXED_PAGE_0, 0xCF, 0x00);
#endif

#ifdef DEV_SUPPORT_EDID
	DoEdidRead();
#endif

#ifdef READKSV
	ReadModifyWriteTPI(0xBB, 0x08, 0x08);
#endif

	if (IsHDMI_Sink())              // select output mode (HDMI/DVI) according to sink capabilty
	{
		TPI_DEBUG_PRINT (("HDMI Sink)\n"));
		ReadModifyWriteTPI(TPI_SYSTEM_CONTROL_DATA_REG, OUTPUT_MODE_MASK, OUTPUT_MODE_HDMI);
	}
	else
	{
		TPI_DEBUG_PRINT (("DVI Sink)\n"));
		ReadModifyWriteTPI(TPI_SYSTEM_CONTROL_DATA_REG, OUTPUT_MODE_MASK, OUTPUT_MODE_DVI);
	}

	//zhy - for when TV power down, 9024 power up, then power up TV ,there is no video #if defined SiI9232_OR_SiI9236
	OnDownstreamRxPoweredUp();		// RX power not determinable? Force to on for now.
	//zhy - #endif
}

void OnHdmiCableDisconnected(void) {

	TPI_DEBUG_PRINT (("HDMI Disconnected\n"));

	hdmiCableConnected = FALSE;

#ifdef DEV_SUPPORT_EDID
	edidDataValid = FALSE;
#endif

	OnDownstreamRxPoweredDown();
	TxPowerState(TX_POWER_STATE_D3);
}


static void OnDownstreamRxPoweredDown(void) {

	TPI_DEBUG_PRINT (("DSRX -> Powered Down\n"));
	dsRxPoweredUp = FALSE;

#ifdef DEV_SUPPORT_HDCP
	HDCP_Off();
#endif

	DisableTMDS();
}


static void OnDownstreamRxPoweredUp(void) {

	TPI_DEBUG_PRINT (("DSRX -> Powered Up\n"));
	dsRxPoweredUp = TRUE;

	HotPlugService();

	pvid_mode = vid_mode;
}

