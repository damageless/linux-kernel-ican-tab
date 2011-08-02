/***********************************************************************************/
/*  Copyright (c) 2002-2009, Silicon Image, Inc.  All rights reserved.             */
/*  No part of this work may be reproduced, modified, distributed, transmitted,    */
/*  transcribed, or translated into any language or computer format, in any form   */
/*  or by any means without written permission of: Silicon Image, Inc.,            */
/*  1060 East Arques Avenue, Sunnyvale, California 94085                           */
/***********************************************************************************/
#include "SIIdefs.h"
#include "SIITypeDefs.h"
#include "SIIConstants.h"
//#include "AMF_Lib.h"


unsigned char CmdTableIndex;                 // Command Table index
unsigned char vid_mode;


// CEC Globals
//============
unsigned int CEC_DeviceList[16];                // List of logical addresses and their status (taken/free)
unsigned char CEC_Initiator;                 // Logical address of THIS device


// Patches
//========
unsigned char EmbeddedSynPATCH;


//UART
//====
unsigned char TXBusy;

unsigned char IDX_InChar;
unsigned char NumOfArgs;
unsigned int F_SBUF_DataReady;
unsigned int F_CollectingData;



#ifndef DEBUG_EDID
unsigned int F_IgnoreEDID;                  // Allow setting of any video input format regardless of Sink's EDID (for debuggin puroses)
#endif

unsigned char pvid_mode;
unsigned int dvi_mode;

// Checksums
unsigned char g_audio_Checksum;	// Audio checksum

