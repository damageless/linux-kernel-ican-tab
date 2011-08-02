/***********************************************************************************/
/*  Copyright (c) 2002-2009, Silicon Image, Inc.  All rights reserved.             */
/*  No part of this work may be reproduced, modified, distributed, transmitted,    */
/*  transcribed, or translated into any language or computer format, in any form   */
/*  or by any means without written permission of: Silicon Image, Inc.,            */
/*  1060 East Arques Avenue, Sunnyvale, California 94085                           */
/***********************************************************************************/

//typedef unsigned int unsigned int;

extern unsigned char CmdTableIndex;                  // Command Table index
extern unsigned char vid_mode;

// CEC Globals
//============

extern unsigned int CEC_DeviceList[];              // List of logical addresses and their status (taken/free)
extern unsigned char CEC_Initiator;                  // Logical address of THIS device

// Patches
//========
extern unsigned char EmbeddedSynPATCH;

//UART
//====
extern unsigned char TXBusy;

extern unsigned char IDX_InChar;
extern unsigned char NumOfArgs;
extern unsigned int F_SBUF_DataReady;
extern unsigned int F_CollectingData;


#ifndef DEBUG_EDID
extern unsigned int F_IgnoreEDID;                   // Allow setting of any video input format regardless of Sink's EDID (for debuggin puroses)
#endif

extern unsigned char pvid_mode;
extern unsigned int dvi_mode;

extern unsigned char g_audio_Checksum;	// Audio checksum

