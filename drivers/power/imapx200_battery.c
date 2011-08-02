/***************************************************************************** 
** XXX driver/power/imapx200_battery.c XXX
** 
** Copyright (c) 2009~2014 ShangHai Infotm Ltd all rights reserved. 
** 
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
** 
** Description: imapx200 ADC based battery monitor driver.
**
** Author:
**     Warits   <warits.wang@infotmic.com.cn>
**      
** Revision History: 
** ----------------- 
** 1.1  XXX 04/06/2010 XXX	Warits
*****************************************************************************/

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/ioport.h>

#include <asm/io.h>

#include "imapx200_battery.h"

#define fdelay(x) do {\
	volatile int __a;	\
	int __i;			\
	for(__i = 0; __i < x; __i++) __a++;	\
} while (0)

static uint32_t CalcRealVot(uint32_t powerVot)
{
	uint32_t realVot;

	realVot = 5000 * powerVot / 255;

	return realVot;
}

static uint32_t GetVoltage(int type)
{
	uint32_t powerVot1, powerVot2;
	uint32_t powerVal[16];
	uint32_t regVal;
	int i;
	
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
		printk(KERN_INFO "%s the return value is %x\n",
		   __func__, powerVot1);
		return powerVot1;
	}
	else
		return 0;
}

static uint32_t GetPowerVot(void)
{
	uint32_t powerVot;
	uint32_t powerVotAvg = 0;
	uint32_t j;
	uint32_t realVot;
	int cnt = 0;

	for (j = 0; j < 10; j++)
	{
		powerVot = GetVoltage(0);
		if (powerVot)
		{
			powerVotAvg += powerVot;
			cnt++;
		}
	}

	if (cnt)
	  powerVotAvg /= cnt;

	realVot = CalcRealVot(powerVotAvg);

	printk(KERN_INFO "ADC0832 read %d times average val %d\n", cnt, powerVotAvg);
	printk(KERN_INFO "Power Vot: Average voltage %d\n", realVot);
	return realVot;
}

static uint32_t IsChargingFull(void)
{
	uint32_t reg_val;
	uint32_t ret;

	reg_val = readl(rGPECON);
	reg_val &= ~(0x3 << 28);
	writel(reg_val, rGPECON);
	reg_val = readl(rGPEPUD);
	reg_val &= ~(0x1 << 14);
	writel(reg_val, rGPEPUD);
	ret = (readl(rGPEDAT) & 0x4000);
	printk(KERN_INFO "Is Charging Full? ret 0x%x\n", ret);

	return ret;
}

static int imap_bat_get_type(void)
{
	int type = 0, i;
	uint32_t powerVot;
	uint32_t powerVotAvg = 0;
	uint32_t realVot;
	int cnt = 0;

	for (i = 0; i < 10; i++)
	{
		powerVot = GetVoltage(1);
		if (powerVot)
		{
			powerVotAvg += powerVot;
			cnt++;
		}
	}
	
	if (cnt)
	  powerVotAvg /= cnt;

	realVot = CalcRealVot(powerVotAvg);
	printk("realV is %d\n, mark is %d\n", realVot, POWER_TYPE_MARK);
	if(realVot >= POWER_TYPE_MARK)
	  type |= BAT_STAT_AC;

	return type;
}

static int imap_bat_get_status(void)
{
	volatile uint32_t votFlag;
	uint32_t batVot = GetPowerVot();
	static int a = 0;

	votFlag = 0;

#if 0
	if(imap_bat_get_type())
	{
		// TODO: battery charging
		if (IsChargingFull())
		  votFlag = iBAT_STAT_FULL;
		else
		  votFlag = iBAT_STAT_CHARGING;
	}
	else
	{
		votFlag = iBAT_STAT_NOTCHARGING;
	}
#endif
	if(a = !a)
	  votFlag = iBAT_STAT_NOTCHARGING;
	else
	  votFlag = iBAT_STAT_CHARGING;

	return votFlag;
}

static int imap_bat_get_present(void)
{
	uint32_t batVot = GetPowerVot();

	if(batVot)
	  return BAT_STAT_PRESENT;

	printk(KERN_INFO "Checking present, batVot is %d\n", batVot);
	return 0;
}


static int imap_bat_get_health(union power_supply_propval *val)
{

	val->intval = POWER_SUPPLY_HEALTH_GOOD;
	return 0;
}

static int imap_bat_get_mfr(union power_supply_propval *val)
{

	val->strval = "Rockie";
	return 0;
}

static int imap_bat_get_capacity(void)
{
	uint32_t lifePercent;
	uint32_t batVot = GetPowerVot();

	lifePercent = 0;

	printk(KERN_INFO "batV is %d, MAX is %d\n", batVot, MAX_USE_VOT);
	if (batVot > MAX_USE_VOT)
	  lifePercent = 100;
	else if (batVot > POWER_VOT_LOW)
	  lifePercent = ((batVot - POWER_VOT_LOW) * 100 * 14 / (MAX_USE_VOT - POWER_VOT_LOW) + 100) / 15;
	else
	  lifePercent = (batVot - MIN_USE_VOT) * 100 / ((POWER_VOT_LOW - MIN_USE_VOT) * 15);

	return lifePercent;
}

static int imap_bat_get_tech(union power_supply_propval *val)
{
	val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
	return 0;
}

static int imap_bat_get_prop(struct power_supply *psy,
   enum power_supply_property psp,
   union power_supply_propval *val)
{
	int ret = 0;

	switch (psp)
	{
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = imap_bat_get_status();
			return 0;
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = imap_bat_get_present();
			break;

		case POWER_SUPPLY_PROP_HEALTH:
			ret = imap_bat_get_health(val);
			break;
		case POWER_SUPPLY_PROP_MANUFACTURER:
			ret = imap_bat_get_mfr(val);
			if (ret)
			  return ret;
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			ret = imap_bat_get_tech(val);
			if (ret)
			  return ret;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_AVG:
			val->intval = GetPowerVot();
			break;
		case POWER_SUPPLY_PROP_CURRENT_AVG:
			val->intval = 0;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = imap_bat_get_capacity();
			break;
		case POWER_SUPPLY_PROP_TEMP:
			val->intval = 50;
			break;
		case POWER_SUPPLY_PROP_TEMP_AMBIENT:
			val->intval = 50;
			break;
		case POWER_SUPPLY_PROP_CHARGE_COUNTER:
			val->intval = 10;
			break;
		case POWER_SUPPLY_PROP_SERIAL_NUMBER:
			break;
		default:
			ret = -EINVAL;
			break;
	}
	return ret;
}

static int imap_ac_get_prop(struct power_supply *psy,
   enum power_supply_property psp,
   union power_supply_propval *val)
{

	switch (psp)
	{
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = imap_bat_get_type();
			break;
		default:
			break;
	}
	return 0;
}


static enum power_supply_property imap_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_AMBIENT,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
};

/*********************************************************************
 * *   Initialisation
 * *********************************************************************/

static struct platform_device *bat_pdev;

static struct power_supply imap_bat =
{
	.properties = imap_bat_props,
	.num_properties = ARRAY_SIZE(imap_bat_props),
	.get_property = imap_bat_get_prop,
	.use_for_apm = 1,
};

static enum power_supply_property imap_ac_props[] =
{
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply imap_ac =
{
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = imap_ac_props,
	.num_properties = ARRAY_SIZE(imap_ac_props),
	.get_property = imap_ac_get_prop,
};

static int __init imap_bat_init(void)
{
	int ret = 0;

	bat_pdev = platform_device_register_simple("battery", 0, NULL, 0);

	ret = power_supply_register(&bat_pdev->dev, &imap_ac);
	if (ret)
	  goto ac_failed;

	imap_bat.name = bat_pdev->name;

	ret = power_supply_register(&bat_pdev->dev, &imap_bat);
	if (ret)
	  goto battery_failed;

	goto success;

	power_supply_unregister(&imap_bat);
battery_failed:
	power_supply_unregister(&imap_ac);
ac_failed:
	platform_device_unregister(bat_pdev);
success:
	return ret;
}

static void __exit imap_bat_exit(void)
{
	power_supply_unregister(&imap_bat);
	power_supply_unregister(&imap_ac);
	platform_device_unregister(bat_pdev);
}

module_init(imap_bat_init);
module_exit(imap_bat_exit);

MODULE_AUTHOR("warits <warits.wang@infotmic.com.cn>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Battery driver for iMAPx200");
