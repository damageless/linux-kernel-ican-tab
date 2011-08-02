/*
 * * Fake Battery driver for android
 * *
 * * Copyright © 2009 Rockie Cheng <aokikyon@gmail.com>
 * *
 * * This program is free software; you can redistribute it and/or modify
 * * it under the terms of the GNU General Public License version 2 as
 * * published by the Free Software Foundation.
 * */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <mach/imapx_gpio.h>
#include <mach/imapx_base_reg.h>
#include <linux/io.h>
#include <asm/delay.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <mach/imapx_gpio.h>

#include "tlv0832.h"

#define fdelay(x) do {\
	volatile int __a;       \
	int __i;			\
	for(__i = 0; __i < x; __i++) __a++;     \
} while (0)

int tlv0832_get_adc_val(void)
{
	uint32_t powerVot1, powerVot2;
	uint32_t powerVal[16];
	uint32_t regVal;
	int i;
	int type = 0;
	regVal = 0;

	// ADC chanel 1 surpose to be power type jurge channel
	SOME_SET(regVal);

	ADC_CS_HIGH(regVal);
	ADC_DI_HIGH(regVal);
	ADC_CLK_LOW(regVal);
	ADC_CS_LOW(regVal);

	//Pull up DI to form 'Start Signal'
	ADC_DI_HIGH(regVal);	//1, START BIT
	fdelay(280);

	//First clock
	ADC_CLK_HIGH(regVal);
	fdelay(280);
	ADC_CLK_LOW(regVal);

	ADC_DI_HIGH(regVal);	//1, SGL/DIF

	fdelay(280);

	//Second clock
	ADC_CLK_HIGH(regVal);
	fdelay(280);
	ADC_CLK_LOW(regVal);

	if (type)
	{
		ADC_DI_HIGH(regVal);	//1, ODD/SIGN
	}
	else
	{
		ADC_DI_LOW(regVal);		//1, ODD/SIGN
	}

	fdelay(280);

	//Third clock
	ADC_CLK_HIGH(regVal);
	fdelay(280);
	ADC_CLK_LOW(regVal);
		
	for(i = 0; i < 16; i++)
	{			
		fdelay(280);
		ADC_CLK_HIGH(regVal);
#if defined (CONFIG_IMAPX_ADC_PRODUCT)
		powerVal[i] = readl(rGPEDAT) & (0x01 << 5);
#else
		powerVal[i] = readl(rGPEDAT) & (0x01 << 4);
#endif
		fdelay(280);
		ADC_CLK_LOW(regVal);
	}

	ADC_CS_HIGH(regVal);
	ADC_DI_HIGH(regVal);

	// merge power value to voltage
	powerVot1 = 0;
	for(i = 1; i < 9; i++)
	{
		if(powerVal[i] != 0)
			powerVot1 += 1 << (8 - i);
	}
	powerVot2 = 0;
	for(i = 8; i < 16; i++)
	{
		if(powerVal[i] != 0)
			powerVot2 += 1 << (i - 8);
	}

	if (powerVot1 == powerVot2)
	{
		//printk( "%s the return value is %x\n",
		 //  __func__, powerVot1);
		return powerVot1;
	}
	else
		return 0;
}

static int __init tlv0832_init(void)
{
	__imapx_register_batt(tlv0832_get_adc_val);
	return 0;
}
module_init(tlv0832_init);

static void __exit tlv0832_exit(void)
{

}
module_exit(tlv0832_exit);

MODULE_AUTHOR("Rockie Cheng <aokikyon@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Fake Battery driver for android");
