/***********************************************************************************/
/*  Copyright (c) 2002-2009, Silicon Image, Inc.  All rights reserved.             */
/*  No part of this work may be reproduced, modified, distributed, transmitted,    */
/*  transcribed, or translated into any language or computer format, in any form   */
/*  or by any means without written permission of: Silicon Image, Inc.,            */
/*  1060 East Arques Avenue, Sunnyvale, California 94085                           */
/***********************************************************************************/
#include "SIIdefs.h"
#include <linux/delay.h>
#include <linux/time.h>
#include "SIITypeDefs.h"
//#include "AMF_Lib.h"
#include "SIIConstants.h"
#include "SIITPI_Regs.h"
#include "Externals.h"
#include "SIIMacros.h"
#include "SIITPI_Access.h"
#include "SIITPI.h"

#define T_DDC_ACCESS    50

unsigned char I2C_ReadByte(unsigned char IICAddr, unsigned char ByteAddr);
void I2C_WriteByte(unsigned char IICAddr, unsigned char ByteAddr, unsigned char Data);


unsigned char I2C_ReadSegmentBlock(unsigned char deviceID, unsigned char segment, unsigned char offset, unsigned char *buffer, unsigned int length)
{
//	I2C_WriteByte(deviceID , 0x01, segment);
	I2C_ReadBlock(deviceID, offset, buffer, length);
	
	return 0;
}

int I2C_ReadBlock(unsigned char IICAddr, unsigned char ByteAddr, unsigned char *Data, int Size)
{
	struct i2c_adapter *adapter;
	struct i2c_msg msgs[] = { 
		{
			.addr   = IICAddr,
			.flags  = 0,
			.len            = 1,
			.buf            = &ByteAddr,
		},
		{
			.addr   = IICAddr,
			.flags  = I2C_M_RD,
			.len            = Size,
			.buf            = Data,
		}
	};

	adapter = i2c_get_adapter(CONFIG_HDMI_SI9022_I2C + 1);
	if (!adapter)
	{
		printk(KERN_ERR "[DDC_Read]: can't get i2c adapter\n");

		return -1; 
	}

	if(i2c_transfer(adapter, msgs, 2) != 2)
	{
		printk("[funciton:%s-line%d] i2c_transfer error\n",__func__,__LINE__);
		return 0;
	}

	return 1;
}

void I2C_WriteBlock(unsigned char IICAddr, unsigned char ByteAddr, unsigned char *Data, int Size)
{
	struct i2c_adapter *adapter;
	unsigned char *buf = kmalloc(Size + 1, GFP_KERNEL);
	unsigned int i;

	buf[0] = ByteAddr;
	for(i=0;i<Size;i++)
		buf[i + 1] = Data[i];

	struct i2c_msg msgs[] = { 
		{
			.addr   = IICAddr,
			.flags  = 0,
			.len            = Size + 1,
			.buf            = buf,
		}
	};

	if (!buf)
	{
		printk(KERN_ERR "[IIC_Write]: unable to allocate memory for EDID.\n");
	}


	adapter = i2c_get_adapter(CONFIG_HDMI_SI9022_I2C + 1);
	if (!adapter)
	{
		printk(KERN_ERR "[IIC_Write]: can't get i2c adapter\n");
	}

	i2c_transfer(adapter, msgs, 1);

	kfree(buf);
}


unsigned char I2C_ReadByte(unsigned char IICAddr, unsigned char ByteAddr)
{
	struct i2c_adapter *adapter;
	unsigned char *buf = kmalloc(sizeof(unsigned char), GFP_KERNEL);
	struct timeval time;

	struct i2c_msg msgs[] = { 
		{
			.addr   = IICAddr,
			.flags  = 0,
			.len            = 1,
			.buf            = &ByteAddr,
		},{
			.addr   = IICAddr,
			.flags  = I2C_M_RD,
			.len            = sizeof(unsigned char),
			.buf            = buf,
		}
	};

	if (!buf)
	{
		printk(KERN_ERR "[IIC_Read]: unable to allocate memory for Read.\n");
		return -1; 
	}

	adapter = i2c_get_adapter(CONFIG_HDMI_SI9022_I2C + 1);
	if (!adapter)
	{
		printk(KERN_ERR "[IIC_Read]: can't get i2c adapter\n");

		return -1; 
	}

	if (i2c_transfer(adapter, msgs, 2) != 2)
	{
		return *buf;
	}

	return *buf;
}

void I2C_WriteByte_NoAck(unsigned char IICAddr, unsigned char ByteAddr, unsigned char Data)
{
	struct i2c_adapter *adapter;
	unsigned char *buf = kmalloc(2 * sizeof(unsigned char), GFP_KERNEL);

	buf[0] = ByteAddr;
	buf[1] = Data;

	struct i2c_msg msgs[] = { 
		{
			.addr   = IICAddr,
			.flags  = I2C_M_IGNORE_NAK,
			.len            = 2 * sizeof(unsigned char),
			.buf            = buf,
		}
	};

	if (!buf)
	{
		printk(KERN_ERR "[IIC_Write]: unable to allocate memory for EDID.\n");
	}


	adapter = i2c_get_adapter(CONFIG_HDMI_SI9022_I2C + 1);
	if (!adapter)
	{
		printk(KERN_ERR "[IIC_Write]: can't get i2c adapter\n");

	}

	i2c_transfer(adapter, msgs, 1);

	kfree(buf);
}


void I2C_WriteByte(unsigned char IICAddr, unsigned char ByteAddr, unsigned char Data)
{
	struct i2c_adapter *adapter;
	unsigned char *buf = kmalloc(2 * sizeof(unsigned char), GFP_KERNEL);
	struct timeval time;

	buf[0] = ByteAddr;
	buf[1] = Data;

	struct i2c_msg msgs[] = { 
		{
			.addr   = IICAddr,
			.flags  = 0,
			.len            = 2 * sizeof(unsigned char),
			.buf            = buf,
		}
	};

	if (!buf)
	{
		printk(KERN_ERR "[IIC_Write]: unable to allocate memory for EDID.\n");
	}


	adapter = i2c_get_adapter(CONFIG_HDMI_SI9022_I2C + 1);
	if (!adapter)
	{
		printk(KERN_ERR "[IIC_Write]: can't get i2c adapter\n");

	}

	i2c_transfer(adapter, msgs, 1);

	kfree(buf);
}

//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION     :   ReadByteTPI ()
//
// PURPOSE      :   Read one unsigned char from a given offset of the TPI interface.
//
// INPUT PARAMS :   Offset of TPI register to be read; A pointer to the variable
//                  where the data read will be stored
//
// OUTPUT PARAMS:   Data - Contains the value read from the register value
//                  specified as the first input parameter
//
// GLOBALS USED :   None
//
// RETURNS      :   TRUE
//
// NOTE         :   ReadByteTPI() is ported from the PC based FW to the uC
//                  version while retaining the same function interface. This
//                  will save the need to modify higher level I/O functions
//                  such as ReadSetWriteTPI(), ReadClearWriteTPI() etc.
//                  A dummy returned value (always TRUE) is provided for
//                  the same reason
//
//////////////////////////////////////////////////////////////////////////////
unsigned char ReadByteTPI(unsigned char RegOffset)
{
    return I2C_ReadByte(TX_SLAVE_ADDR, RegOffset);
}


//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION      :  WriteByteTPI ()
//
// PURPOSE       :  Write one unsigned char to a given offset of the TPI interface.
//
// INPUT PARAMS  :  Offset of TPI register to write to; value to be written to
//                  that TPI retister
//
// OUTPUT PARAMS :  None
//
// GLOBALS USED  :  None
//
// RETURNS       :  VOID
//
//////////////////////////////////////////////////////////////////////////////
void WriteByteTPI(unsigned char RegOffset, unsigned char Data)
{
    I2C_WriteByte(TPI_BASE_ADDR, RegOffset, Data);
}


//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION      :  ReadSetWriteTPI(unsigned char Offset, unsigned char Pattern)
//
// PURPOSE       :  Write "1" to all bits in TPI offset "Offset" that are set
//                  to "1" in "Pattern"; Leave all other bits in "Offset"
//                  unchanged.
//
// INPUT PARAMS  :  Offset  :   TPI register offset
//                  Pattern :   GPIO bits that need to be set
//
// OUTPUT PARAMS :  None
//
// GLOBALS USED  :  None
//
// RETURNS       :  TRUE
//
//////////////////////////////////////////////////////////////////////////////
void ReadSetWriteTPI(unsigned char Offset, unsigned char Pattern)
{
    unsigned char Tmp;

    Tmp = ReadByteTPI(Offset);

    Tmp |= Pattern;
    WriteByteTPI(Offset, Tmp);
}

//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION      :  ReadClearWriteTPI(unsigned char Offset, unsigned char Pattern)
//
// PURPOSE       :  Write "0" to all bits in TPI offset "Offset" that are set
//                  to "1" in "Pattern"; Leave all other bits in "Offset"
//                  unchanged.
//
// INPUT PARAMS  :  Offset  :   TPI register offset
//                  Pattern :   "1" for each TPI register bit that needs to be
//                              cleared
//
// OUTPUT PARAMS :  None
//
// GLOBALS USED  :  None
//
// RETURNS       :  TRUE
//
//////////////////////////////////////////////////////////////////////////////
void ReadClearWriteTPI(unsigned char Offset, unsigned char Pattern)
{
    unsigned char Tmp;

    Tmp = ReadByteTPI(Offset);

    Tmp &= ~Pattern;
    WriteByteTPI(Offset, Tmp);
}


//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION      :  ReadModifyWriteTPI(unsigned char Offset, unsigned char Mask, unsigned char Value)
//
// PURPOSE       :  Write "Value" to all bits in TPI offset "Offset" that are set
//                  to "1" in "Mask"; Leave all other bits in "Offset"
//                  unchanged.
//
// INPUT PARAMS  :  Offset  :   TPI register offset
//                  Mask    :   "1" for each TPI register bit that needs to be
//                              modified
//					Value   :   The desired value for the register bits in their
//								proper positions
//
// OUTPUT PARAMS :  None
//
// GLOBALS USED  :  None
//
// RETURNS       :  void
//
//////////////////////////////////////////////////////////////////////////////
void ReadModifyWriteTPI(unsigned char Offset, unsigned char Mask, unsigned char Value)
{
    unsigned char Tmp;

    Tmp = ReadByteTPI(Offset);
    Tmp &= ~Mask;
	Tmp |= (Value & Mask);
    WriteByteTPI(Offset, Tmp);
}


////////////////////////////////////////////////////////////////////////////////
//
// FUNCTION         :   ReadBlockTPI ()
//
// PURPOSE          :   Read NBytes from offset Addr of the TPI slave address
//                      into a unsigned char Buffer pointed to by Data
//
// INPUT PARAMETERS :   TPI register offset, number of unsigned chars to read and a
//                      pointer to the data buffer where the data read will be
//                      saved
//
// OUTPUT PARAMETERS:   pData - pointer to the buffer that will store the TPI
//                      block to be read
//
// RETURNED VALUE   :   VOID
//
// GLOBALS USED     :   None
//
////////////////////////////////////////////////////////////////////////////////
void ReadBlockTPI(unsigned char TPI_Offset, unsigned int NBytes, unsigned char * pData)
{
    I2C_ReadBlock(TPI_BASE_ADDR, TPI_Offset, pData, NBytes);
}


////////////////////////////////////////////////////////////////////////////////
//
// FUNCTION         :   WriteBlockTPI ()
//
// PURPOSE          :   Write NBytes from a unsigned char Buffer pointed to by Data to
//                      the TPI I2C slave starting at offset Addr
//
// INPUT PARAMETERS :   TPI register offset to start writing at, number of unsigned chars
//                      to write and a pointer to the data buffer that contains
//                      the data to write
//
// OUTPUT PARAMETERS:   None.
//
// RETURNED VALUES  :   void
//
// GLOBALS USED     :   None
//
////////////////////////////////////////////////////////////////////////////////
void WriteBlockTPI(unsigned char TPI_Offset, unsigned int NBytes, unsigned char * pData)
{
    I2C_WriteBlock(TPI_BASE_ADDR, TPI_Offset, pData, NBytes);
}


#if defined HAS_CTRL_BUS
unsigned char ReadByteCBUS (unsigned char Offset) {
	return I2C_ReadByte(CBUS_SLAVE_ADDR, Offset);
	}

void WriteByteCBUS(unsigned char Offset, unsigned char Data) {
	I2C_WriteByte(CBUS_SLAVE_ADDR, Offset, Data);
	}
#endif

//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION      :  GetDDC_Access(void)
//
// PURPOSE       :  Request access to DDC bus from the receiver
//
// INPUT PARAMS  :  None
//
// OUTPUT PARAMS :  None
//
// GLOBALS USED  :  None
//
// RETURNS       :  TRUE if bus obtained successfully. FALSE if failed.
//
//////////////////////////////////////////////////////////////////////////////

bool GetDDC_Access (unsigned char* SysCtrlRegVal)
{
	unsigned char sysCtrl;
	unsigned char DDCReqTimeout = T_DDC_ACCESS;
	unsigned char TPI_ControlImage;

	TPI_TRACE_PRINT((">>GetDDC_Access()\n"));

	sysCtrl = ReadByteTPI (TPI_SYSTEM_CONTROL_DATA_REG);			// Read and store original value. Will be passed into ReleaseDDC()
	*SysCtrlRegVal = sysCtrl;

	sysCtrl |= DDC_BUS_REQUEST_REQUESTED;
	WriteByteTPI (TPI_SYSTEM_CONTROL_DATA_REG, sysCtrl);

	while (DDCReqTimeout--)											// Loop till 0x1A[1] reads "1"
	{
		TPI_ControlImage = ReadByteTPI(TPI_SYSTEM_CONTROL_DATA_REG);

		if (TPI_ControlImage & DDC_BUS_GRANT_MASK)					// When 0x1A[1] reads "1"
		{
			sysCtrl |= DDC_BUS_GRANT_GRANTED;
			WriteByteTPI(TPI_SYSTEM_CONTROL_DATA_REG, sysCtrl);		// lock host DDC bus access (0x1A[2:1] = 11)
			return TRUE;
		}
		WriteByteTPI(TPI_SYSTEM_CONTROL_DATA_REG, sysCtrl);			// 0x1A[2] = "1" - Requst the DDC bus
		DelayMS(200);
	}

	WriteByteTPI(TPI_SYSTEM_CONTROL_DATA_REG, sysCtrl);				// Failure... restore original value.
	return FALSE;
}


//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION      :  ReleaseDDC(void)
//
// PURPOSE       :  Release DDC bus
//
// INPUT PARAMS  :  None
//
// OUTPUT PARAMS :  None
//
// GLOBALS USED  :  None
//
// RETURNS       :  TRUE if bus released successfully. FALSE if failed.
//
//////////////////////////////////////////////////////////////////////////////

bool ReleaseDDC(unsigned char SysCtrlRegVal)
{
	unsigned char DDCReqTimeout = T_DDC_ACCESS;
	unsigned char TPI_ControlImage;

	TPI_TRACE_PRINT((">>ReleaseDDC()\n"));

	SysCtrlRegVal &= ~BITS_2_1;					// Just to be sure bits [2:1] are 0 before it is written

	while (DDCReqTimeout--)						// Loop till 0x1A[1] reads "0"
	{
		// Cannot use ReadClearWriteTPI() here. A read of TPI_SYSTEM_CONTROL is invalid while DDC is granted.
		// Doing so will return 0xFF, and cause an invalid value to be written back.
		//ReadClearWriteTPI(TPI_SYSTEM_CONTROL,BITS_2_1); // 0x1A[2:1] = "0" - release the DDC bus

//		WriteByteTPI(TPI_SYSTEM_CONTROL_DATA_REG, SysCtrlRegVal);
		I2C_WriteByte_NoAck(TPI_BASE_ADDR,TPI_SYSTEM_CONTROL_DATA_REG, SysCtrlRegVal);
		TPI_ControlImage = ReadByteTPI(TPI_SYSTEM_CONTROL_DATA_REG);

		if (!(TPI_ControlImage & BITS_2_1))		// When 0x1A[2:1] read "0"
			return TRUE;
	}

	return FALSE;								// Failed to release DDC bus control
}


////////////////////////////////////////////////////////////////////////////////
//
// FUNCTION         :   ReadBlockHDCP ()
//
// PURPOSE          :   Read NBytes from offset Addr of the HDCP slave address
//                      into a unsigned char Buffer pointed to by Data
//
// INPUT PARAMETERS :   HDCP port offset, number of unsigned chars to read and a
//                      pointer to the data buffer where the data read will be
//                      saved
//
// OUTPUT PARAMETERS:   Data, a buffer that contains the block read from the CPI
//
// RETURNED VALUE   :   void
//
// GLOBALS USED     :   None
//
////////////////////////////////////////////////////////////////////////////////
#ifdef READKSV
void ReadBlockHDCP(unsigned char TPI_Offset, unsigned int NBytes, unsigned char * pData)
{
	TPI_TRACE_PRINT((">>ReadBlockHDCP()\n"));
    I2C_ReadBlock(HDCP_SLAVE_ADDR, TPI_Offset, pData, NBytes);
}
#endif


#define TPI_INTERNAL_PAGE_REG	0xBC
#define TPI_INDEXED_OFFSET_REG	0xBD
#define TPI_INDEXED_VALUE_REG	0xBE


//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION     :   ReadIndexedRegister()
//
// PURPOSE      :   Read an indexed register value
//
//                  Write:
//                      1. 0xBC => Internal page num
//                      2. 0xBD => Indexed register offset
//
//                  Read:
//                      3. 0xBE => Returns the indexed register value
//
// INPUT PARAMS :   Internal page number, indexed register offset, pointer to
//                  buffer to store read value
//
// OUTPUT PARAMS:   Buffer that stores the read value
//
// GLOBALS USED :   None
//
// RETURNS      :   TRUE 
//
//////////////////////////////////////////////////////////////////////////////
#ifdef DEV_INDEXED_READ
unsigned char ReadIndexedRegister(unsigned char PageNum, unsigned char RegOffset) {
    WriteByteTPI(TPI_INTERNAL_PAGE_REG, PageNum);		// Internal page
    WriteByteTPI(TPI_INDEXED_OFFSET_REG, RegOffset);	// Indexed register
    return ReadByteTPI(TPI_INDEXED_VALUE_REG); 		// Return read value
}
#endif


//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION     :   WriteIndexedRegister()
//
// PURPOSE      :   Write a value to an indexed register
//
//                  Write:
//                      1. 0xBC => Internal page num
//                      2. 0xBD => Indexed register offset
//                      3. 0xBE => Set the indexed register value
//
// INPUT PARAMS :   Internal page number, indexed register offset, value
//                  to write to indexed register
//
// OUTPUT PARAMS:   None
//
// GLOBALS USED :   None
//
// RETURNS      :   TRUE 
//
//////////////////////////////////////////////////////////////////////////////
#ifdef DEV_INDEXED_WRITE
void WriteIndexedRegister(unsigned char PageNum, unsigned char RegOffset, unsigned char RegValue) {
    WriteByteTPI(TPI_INTERNAL_PAGE_REG, PageNum);  // Internal page
    WriteByteTPI(TPI_INDEXED_OFFSET_REG, RegOffset);  // Indexed register
    WriteByteTPI(TPI_INDEXED_VALUE_REG, RegValue);    // Read value into buffer
}
#endif

//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION      :  ReadModifyWriteIndexedRegister(unsigned char PageNum, unsigned char Offset, unsigned char Mask, unsigned char Value)
//
// PURPOSE       :  Write "Value" to all bits in TPI offset "Offset" that are set
//                  to "1" in "Mask"; Leave all other bits in "Offset"
//                  unchanged.
//
// INPUT PARAMS  :  Offset  :   TPI register offset
//                  Mask    :   "1" for each TPI register bit that needs to be
//                              modified
//					Value   :   The desired value for the register bits in their
//								proper positions
//
// OUTPUT PARAMS :  None
//
// GLOBALS USED  :  None
//
// RETURNS       :  void
//
//////////////////////////////////////////////////////////////////////////////
#ifdef DEV_INDEXED_RMW
void ReadModifyWriteIndexedRegister(unsigned char PageNum, unsigned char RegOffset, unsigned char Mask, unsigned char Value)
{
    unsigned char Tmp;

    WriteByteTPI(TPI_INTERNAL_PAGE_REG, PageNum);
    WriteByteTPI(TPI_INDEXED_OFFSET_REG, RegOffset);
    Tmp = ReadByteTPI(TPI_INDEXED_VALUE_REG);

    Tmp &= ~Mask;
	Tmp |= (Value & Mask);

    WriteByteTPI(TPI_INDEXED_VALUE_REG, Tmp);
}
#endif

//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION     :   SetPLL()
//
// PURPOSE      :   Set the 9024/9024/9222 chip PLL to multiply the input pixel
//                  clock by the value passed to this function as a parameter
//                  registers:
//
// INPUT PARAMS :   PLL Multiplication factor (represents 0.5, 1, 2 or 4)
//
// OUTPUT PARAMS:   None
//
// GLOBALS USED :   None
//
// RETURNS      :   TRUE 
//
//////////////////////////////////////////////////////////////////////////////
#ifdef DEV_INDEXED_PLL
bool SetPLL(unsigned char TClkSel)
{
    unsigned char RegValue;
    unsigned char Pattern;

    // Set up register 0x82[6:5] (same for all input pixel frequencies for a specific multiplier value):

    ReadIndexedRegister(0x01, TMDS_CONT_REG, &RegValue);

    Pattern = TClkSel << 5;
    RegValue &= ~BITS_6_5;
    RegValue |= Pattern;

    WriteByteTPI(TPI_REGISTER_VALUE_REG, RegValue);

    return TRUE;
}
#endif


//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION     :   SetChannelLayout()
//
// PURPOSE      :   Set up the Channel layout field of internal register 0x2F
//                  (0x2F[1])
//
// INPUT PARAMS :   Number of audio channels: "0 for 2-Channels ."1" for 8.
//
// OUTPUT PARAMS:   None
//
// GLOBALS USED :   None
//
// RETURNS      :   TRUE 
//
//////////////////////////////////////////////////////////////////////////////
#ifdef SetChannelLayout
bool SetChannelLayout(unsigned char Count)
{
    // Indexed register 0x7A:0x2F[1]:
    WriteByteTPI(TPI_INTERNAL_PAGE_REG, 0x02); // Internal page 2
    WriteByteTPI(TPI_REGISTER_OFFSET_REG, 0x2F);

    Count &= THREE_LSBITS;

    if (Count == TWO_CHANNEL_LAYOUT)
    {
        // Clear 0x2F:
        ReadClearWriteTPI(TPI_REGISTER_VALUE_REG, BIT_1);
    }

    else if (Count == EIGHT_CHANNEL_LAYOUT)
    {
        // Set 0x2F[0]:
        ReadSetWriteTPI(TPI_REGISTER_VALUE_REG, BIT_1);
    }

    return TRUE;
}
#endif

//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION     :   SetInputWordLength()
//
// PURPOSE      :   Tell the Tx what the I2S input unsigned int is
//
// INPUT PARAMS :   Input unsigned int length code per SiI-PR-1032-0.02, table 20
//
// OUTPUT PARAMS:   None
//
// GLOBALS USED :   None
//
// RETURNS      :   TRUE
//
// NOTE         :   Used by 9022/4 only. 9022A/24A and 9334 set that value
//                  automatically when 0x25[5:3] is set
//
//////////////////////////////////////////////////////////////////////////////
#ifdef SETINPUTWORDLENGTH
bool SetInputWordLength(unsigned char Length)
{
    unsigned char RegValue;

    RegValue = ReadIndexedRegister(0x02, AUDIO_INPUT_LENGTH);

    RegValue &= ~LOW_NIBBLE;
    RegValue |= Length;

    WriteByteTPI(TPI_REGISTER_VALUE_REG, RegValue);

    return TRUE;
}
#endif
