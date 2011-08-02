/***********************************************************************************/
/*  Copyright (c) 2002-2009, Silicon Image, Inc.  All rights reserved.             */
/*  No part of this work may be reproduced, modified, distributed, transmitted,    */
/*  transcribed, or translated into any language or computer format, in any form   */
/*  or by any means without written permission of: Silicon Image, Inc.,            */
/*  1060 East Arques Avenue, Sunnyvale, California 94085                           */
/***********************************************************************************/
#include "imap_HDMI.h"

void DelayMS(int M_Sec);
void OnHdmiCableDisconnected(void);
void OnHdmiCableConnected(LCD_TIMING timing);

bool TPI_Init (void);			// Document purpose, usage
void TPI_Poll (void);			// Document purpose, usage, rename

void EnableTMDS (void);			// Document purpose, usage
void DisableTMDS (void);		// Document purpose, usage

void RestartHDCP (void);		// Document purpose, usage

void SetInputColorSpace (char inputColorSpace);

//void SetAudioMute (char audioMute);
void SetAudioMute (unsigned char audioMute);
void TxPowerState(unsigned char powerState);

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

void HotPlugService (void);		// Why is this being called form outside TPI.c?? Re-architect
//bool EnableInterrupts (char);	// Why is this being called form outside TPI.c?? Re-architect
bool EnableInterrupts(unsigned char Interrupt_Pattern);
extern unsigned char VideoMode[9];

// TPI Firmware Version
//=====================
//static const char TPI_FW_VERSION[] = {'0', '.', '2', '8'};

