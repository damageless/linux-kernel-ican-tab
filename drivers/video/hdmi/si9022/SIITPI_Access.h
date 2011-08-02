/***********************************************************************************/
/*  Copyright (c) 2002-2009, Silicon Image, Inc.  All rights reserved.             */
/*  No part of this work may be reproduced, modified, distributed, transmitted,    */
/*  transcribed, or translated into any language or computer format, in any form   */
/*  or by any means without written permission of: Silicon Image, Inc.,            */
/*  1060 East Arques Avenue, Sunnyvale, California 94085                           */
/***********************************************************************************/

#include <asm/io.h>
#include <linux/i2c.h>
#include <plat/imapx.h>

unsigned char I2C_ReadByte(unsigned char IICAddr, unsigned char ByteAddr);
void I2C_WriteByte(unsigned char IICAddr, unsigned char ByteAddr, unsigned char Data);
unsigned char I2C_ReadSegmentBlock(unsigned char deviceID, unsigned char segment, unsigned char offset, unsigned char *buffer, unsigned int length);
int I2C_ReadBlock(unsigned char IICAddr, unsigned char ByteAddr, unsigned char *Data, int Size);
void I2C_WriteBlock(unsigned char IICAddr, unsigned char ByteAddr, unsigned char *Data, int Size);
unsigned char ReadByteTPI(unsigned char);
void WriteByteTPI(unsigned char, unsigned char);

#if defined HAS_CTRL_BUS
unsigned char ReadByteCBUS (unsigned char Offset);
void WriteByteCBUS(unsigned char Offset, unsigned char Data);
#endif

void ReadSetWriteTPI(unsigned char, unsigned char);
void ReadClearWriteTPI(unsigned char, unsigned char);
void ReadModifyWriteTPI(unsigned char Offset, unsigned char Mask, unsigned char Value);

//void ReadBlockTPI(unsigned char, int, unsigned char *);
void ReadBlockTPI(unsigned char TPI_Offset, unsigned int NBytes, unsigned char * pData);
//void WriteBlockTPI(unsigned char, int, unsigned char *);
void WriteBlockTPI(unsigned char TPI_Offset, unsigned int NBytes, unsigned char * pData);

bool GetDDC_Access(unsigned char* SysCtrlRegVal);
bool ReleaseDDC(unsigned char SysCtrlRegVal);

#ifdef READKSV
void ReadBlockHDCP(unsigned char, int, unsigned char *);
#endif

unsigned char ReadIndexedRegister(unsigned char, unsigned char);
void WriteIndexedRegister(unsigned char, unsigned char, unsigned char);
void ReadModifyWriteIndexedRegister(unsigned char, unsigned char, unsigned char, unsigned char);

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

// Indexed Register Offsets, Constants
//====================================
#define INDEXED_PAGE_0			0x01
#define INDEXED_PAGE_1			0x02
#define INDEXED_PAGE_2			0x03

#define DEVICE_ID_LOW_BYTE      0x02
#define DEVICE_ID_HI_BYTE       0x03
#define AUDIO_INPUT_LENGTH		0x24

#define TMDS_CONT_REG           0x82

#define SW_RESET                0x05
#define POWER_DOWN              0x6F


#define DIV_BY_2                0x00
#define MULT_BY_1               0x01
#define MULT_BY_2               0x02
#define MULT_BY_4               0x03

#ifdef DEV_INDEXED_PLL
bool SetPLL(unsigned char);
#endif

// Prototype Declarations
//=======================

bool SetInputWordLength(unsigned char);

bool SetChannelLayout(unsigned char);
