#ifndef __IMAPX200_BATTERY_H__
#define __IMAPX200_BATTERY_H__

#include <mach/hardware.h>

#define POWER_TYPE_MARK				2000
#define MAX_USE_VOT					3130	// Voltage
#define MIN_USE_VOT						2150	// Voltage
#define POWER_VOT_HIGH					2700
#define POWER_VOT_LOW					2600
#define POWER_VOT_CRITICAL				2350
#define CHARGING_FULL					3190

// ADC registers operations macros
#define ADC_CLK_LOW(pVal)	do {\
	pVal = readl(rGPEDAT); \
	pVal &= ~(0x01 << 6); \
	writel(pVal, rGPEDAT);		\
} while (0)

#define ADC_CLK_HIGH(pVal)	do {\
	pVal = readl(rGPEDAT); \
	pVal |= (0x01 << 6); \
	writel(pVal, rGPEDAT);	\
} while (0)


#if defined (CONFIG_IMAPX_ADC_PRODUCT)
//DI: GPE4
#define ADC_DI_LOW(pVal)	do {\
	pVal = readl(rGPEDAT); \
	pVal &= ~(0x01 << 4); \
	writel(pVal, rGPEDAT);	\
} while (0)

#define ADC_DI_HIGH(pVal)	do {\
	pVal = readl(rGPEDAT); \
	pVal |= (0x01 << 4); \
	writel(pVal, rGPEDAT);	\
} while (0)

//CS: GPO5
#define ADC_CS_LOW(pVal)	do {\
	pVal = readl(rGPODAT); \
	pVal &= ~(0x01 << 5); \
	writel(pVal, rGPODAT);		\
} while (0)

#define ADC_CS_HIGH(pVal)	do {\
	pVal = readl(rGPODAT); \
	pVal |= (0x01 << 5); \
	writel(pVal, rGPODAT);	\
} while (0)

//DI: GPE4<->Output
//DO: GPE5<->Input
//CLK: GPE6<->Output
//CS: GPO5<->Output
#define SOME_SET(pVal)		do {\
	pVal = readl(rGPECON); \
	pVal &= ~(0x3f << 8); \
	pVal |= (0x11 << 8); \
	writel(pVal, rGPECON); \
	pVal = readl(rGPOCON); \
	pVal &= ~(0x3 << 10); \
	pVal |= (0x1 << 10); \
	writel(pVal, rGPOCON);		\
} while (0)

#else
#define ADC_DI_LOW(pVal)	do {	\
	pVal = readl(rGPEDAT); \
	pVal &= ~(0x01 << 5); \
	writel(pVal, rGPEDAT);	\
} while (0)

#define ADC_DI_HIGH(pVal)	do {	\
	pVal = readl(rGPEDAT); \
	pVal |= (0x01 << 5); \
	writel(pVal, rGPEDAT);	\
} while (0)

#define ADC_CS_LOW(pVal)	do {	\
	pVal = readl(rGPEDAT); \
	pVal &= ~(0x01 << 7); \
	writel(pVal, rGPEDAT);	\
} while (0)

#define ADC_CS_HIGH(pVal)	do {	\
	pVal = readl(rGPEDAT); \
	pVal |= (0x01 << 7); \
	writel(pVal, rGPEDAT);	\
} while (0)

#define SOME_SET(pVal)	do {	\
	pVal = readl(rGPECON); \
	pVal &= ~(0xff << 8); \
	pVal |= (0x54 << 8); \
	writel(pVal, rGPECON);	\
} while (0)
#endif

#define BAT_STAT_PRESENT 0x01
#define BAT_STAT_FULL   0x02
#define BAT_STAT_LOW   0x04
#define BAT_STAT_DESTROY 0x08
#define BAT_STAT_AC   0x10
#define BAT_STAT_CHARGING 0x20
#define BAT_STAT_DISCHARGING 0x40

#define iBAT_STAT_CHARGING 0x1
#define iBAT_STAT_FULL   0x04
#define iBAT_STAT_NOTCHARGING 0x3

#define BAT_ERR_INFOFAIL 0x02
#define BAT_ERR_OVERVOLTAGE 0x04
#define BAT_ERR_OVERTEMP 0x05
#define BAT_ERR_GAUGESTOP 0x06
#define BAT_ERR_OUT_OF_CONTROL 0x07
#define BAT_ERR_ID_FAIL   0x09
#define BAT_ERR_ACR_FAIL 0x10

#define BAT_ADDR_MFR_TYPE 0x5F

#endif /* __IMAPX200_BATTERY_H__ */
