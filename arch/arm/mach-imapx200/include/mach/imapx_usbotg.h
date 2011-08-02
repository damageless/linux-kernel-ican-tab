/* Reimported by warits on 20100630 */

#ifndef __IMAP_OTG_H__
#define __IMAP_OTG_H__

#define USB_DEV		(0x4000)
#define USB_LK		(0x8000)

/* OTG USB_egisters */
#define USB_ACR       (USB_DEV + 0x00)  //  Application control Register
#define USB_MDAR      (USB_DEV + 0x04)  //  Master Destination Address Register 
#define USB_UDCR      (USB_DEV + 0x08)  //  Usb and Device Control Register
#define USB_FNCR      (USB_DEV + 0x0C)  //  Frame Number Control Register
#define USB_FHHR      (USB_DEV + 0x10)  //  Flush,handshake and halt Bit Register
#define USB_PRIR      (USB_DEV + 0x14)  //  Post Request Information Register
#define USB_STR0      (USB_DEV + 0x18)  //  Setup Transaction0 Register
#define USB_STR1      (USB_DEV + 0x1C)  //  Setup Transaction1 Register
#define USB_BFCR      (USB_DEV + 0x20)  //  Buffer Flush Control Register
#define USB_TBCR0     (USB_DEV + 0x30)  //  TxBuffer0 ControlRegister
#define USB_TBCR1     (USB_DEV + 0x34)  //  TxBuffer1 ControlRegister
#define USB_TBCR2     (USB_DEV + 0x38)  //  TxBuffer2 ControlRegister
#define USB_TBCR3     (USB_DEV + 0x3C)  //  TxBuffer3 ControlRegister
#define USB_TBCR(x)   (USB_DEV + 0x30 + ((x) << 2))  //  TxBufferx ControlRegister
#define USB_IER       (USB_DEV + 0x50)  //  Interrupt Enable Register
#define USB_IDR       (USB_DEV + 0x54)  //  Interrupt Disable Register
#define USB_ISR       (USB_DEV + 0x58)  //  Interrupt Status Register
#define USB_CCR       (USB_DEV + 0x70)  //  Current Configuration Register
#define USB_PIR0      (USB_DEV + 0x74)  //  Physical Interface Register0
#define USB_PIR1      (USB_DEV + 0x78)  //  Physical Interface Register1
#define USB_EDR0      (USB_DEV + 0x80)  //  EndPoint Descriptor Register0
#define USB_EDR1      (USB_DEV + 0x84)  //  EndPoint Descriptor Register1
#define USB_EDR2      (USB_DEV + 0x88)  //  EndPoint Descriptor Register2
#define USB_EDR3      (USB_DEV + 0x8C)  //  EndPoint Descriptor Register3
#define USB_EDR4      (USB_DEV + 0x90)  //  EndPoint Descriptor Register4
#define USB_EDR5      (USB_DEV + 0x94)  //  EndPoint Descriptor Register5
#define USB_EDR6      (USB_DEV + 0x98)  //  EndPoint Descriptor Register6
#define USB_EDR(x)    (USB_DEV + 0x80 + ((x) << 2))  //  EndPoint Descriptor Register6

/* OTG link USB_egisters */
#define USB_BCWR      (USB_LK + 0x00)  //  OTG Link Write Register
#define USB_BCSR      (USB_LK + 0x10)  //  OTG Link Status Register
#define USB_BCISR     (USB_LK + 0x20)  //  Interrupt Status Register
#define USB_BCIER     (USB_LK + 0x30)  //  Interrupt Enable Register
#define USB_BCIDR     (USB_LK + 0x40)  //  Interrupt Disable Register
#define USB_IPCR      (USB_LK + 0x50)  //  IP Control Register

/* event bits */

#define USB_Connect			(1 << 0)
#define USB_Disconnect		(1 << 1)
#define USB_Reset			(1 << 2)
#define USB_Suspend			(1 << 3)
#define USB_Resume			(1 << 4)
#define USB_SOF				(1 << 5)

#define USB_DMA_Done		(1 << 8)
#define USB_DMA_Error		(1 << 9)
#define USB_Sync_Frame		(1 << 10)
#define USB_SF1				(1 << 11)
#define USB_SF2				(1 << 12)
#define USB_SF3				(1 << 13)
#define USB_Set_Interface	(1 << 14)

#define USB_EP0				(0xf << 16)
#define USB_EP0_Setup		(1 << 16)
#define USB_EP0_OUT			(1 << 17)
#define USB_EP0_IN			(1 << 18)
#define USB_EP0_Query		(1 << 19)

#define USB_PEP				(0x3f << 24)
#define USB_PEP_Tran(x)		(1 << (23 + (x)))

/* UDCR bits */
#define USB_MTMS_			(1)
#define USB_MTMS_MSK		(0x7)
#define USB_DSI				(1 << 18)

/* FHHR bits */
#define USB_PEP_HALT(x)		(1 << (16 + (x)))

/* ACR bits */
#define USB_IN_Prebuffer	(1 << 0)
#define USB_QueryACK		(1 << 1)
#define USB_DMA_RxEn		(1 << 4)
#define USB_DMA_TxEn		(1 << 5)
#define USB_TxBuffer(x)		((x) << 8)
#define USB_ReqLength(x)	((x) << 16)
#define USB_ReqError		(1 << 11)

/* PRIR bits */
#define USB_ReqCnt_MSK		(0xfff)

/* BFCR bits */
#define USB_Flush_TXB(x)	(1 << (4 + (x)))
#define USB_Flush_All		(1 << 0)

/* EDR bits */
#define USB_EP_Type(x)		((x) << 0)
#define USB_EP_DirIn		(1 << 3)
#define USB_LogicNo(x)		((x) << 4)
#define USB_EP_Alt(x)		((x) << 8)
#define USB_EP_Phy(x)		((x) << 12)
#define USB_MaxPacket(x)	((x) << 16)
#define USB_InBuf(x)		((x) << 28)
#define USB_EP_HALT			(1 << 31)

/* BCSR */
#define USB_B_Valid			(1 << 13)
#define USB_DRDB_CS			(3 << 4)

/* BCWR */
#define USB_B_Connect		(1 << 11)

#endif /* __IMAP_OTG_H__ */

