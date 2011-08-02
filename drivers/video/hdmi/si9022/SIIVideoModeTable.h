/***********************************************************************************/
/*  Copyright (c) 2002-2009, Silicon Image, Inc.  All rights reserved.             */
/*  No part of this work may be reproduced, modified, distributed, transmitted,    */
/*  transcribed, or translated into any language or computer format, in any form   */
/*  or by any means without written permission of: Silicon Image, Inc.,            */
/*  1060 East Arques Avenue, Sunnyvale, California 94085                           */
/***********************************************************************************/

typedef struct {
	unsigned char Mode_C1;
	unsigned char Mode_C2;
	unsigned char SubMode;
} ModeIdType;

typedef struct {
	unsigned int Pixels;
	unsigned int Lines;
} PxlLnTotalType;

typedef struct {
	unsigned int H;
	unsigned int V;
} HVPositionType;

typedef struct {
	unsigned int H;
	unsigned int V;
} HVResolutionType;

typedef struct{
   unsigned char RefrTypeVHPol;
   unsigned int VFreq;
   PxlLnTotalType Total;
} TagType;

typedef struct {
	unsigned char IntAdjMode;
	unsigned int HLength;
	unsigned char VLength;
	unsigned int Top;
	unsigned int Dly;
	unsigned int HBit2HSync;
	unsigned char VBit2VSync;
	unsigned int Field2Offset;
}  _656Type;

typedef struct {
	ModeIdType ModeId;
	unsigned int PixClk;
	TagType Tag;
	HVPositionType Pos;
	HVResolutionType Res;
	unsigned char AspectRatio;
	_656Type _656;
	unsigned char PixRep;
	unsigned char _3D_Struct;
} VModeInfoType;

#define NSM                     0   // No Sub-Mode

#define ProgrVNegHNeg           0x00
#define ProgrVNegHPos           0x01
#define ProgrVPosHNeg           0x02
#define ProgrVPosHPos           0x03

#define InterlaceVNegHNeg       0x04
#define InterlaceVPosHNeg       0x05
#define InterlaceVNgeHPos       0x06
#define InterlaceVPosHPos       0x07

#define PC_BASE                 60

// Aspect ratio
//=============
#define _4                      0   // 4:3
#define _4or16                  1   // 4:3 or 16:9
#define _16                     2   // 16:9

//extern code const VModeInfoType VModesTable[];
extern const VModeInfoType VModesTable[];
//extern code const unsigned char AspectRatioTable[];
extern const unsigned char AspectRatioTable[];

unsigned char ConvertVIC_To_VM_Index(unsigned char, unsigned char);
