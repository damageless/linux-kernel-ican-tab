/***************************************************************************** 
** arch/arm/mach-imapx200/hwid.c
** 
** Copyright (c) 2009~2014 ShangHai Infotm Ltd all rights reserved. 
** 
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
** 
** Description: PCB test, module .
**
** Author:
**		Warits <warits.wang@infotmic.com.cn>
**      
** Revision History: 
** ----------------- 
** 1.1  11/30/2010
*****************************************************************************/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/sysdev.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <asm/hardware/iomd.h>
#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <asm/proc-fns.h>
#include <asm/cpu-single.h>
#include <plat/clock.h>
#include <plat/cpu.h>
#include <plat/devs.h>
#include <plat/imapx200.h>
#include <mach/idle.h>
#include <plat/imapx.h>
#include <plat/regs-serial.h>
#include <plat/pm.h>
#include <linux/amba/bus.h>

#include <mach/imap_hwid.h>

static struct product_data imap_product_data [] = {
	/* id     sd  na  ph  rf  gs  ca  hd  ad  gp  lc  au  no  wi  tp  ba */

	/* wwe8b: camera changed */
	{"7.1.5",  1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  1,  1,  1,  1,  1},

	/* wwe10b */
	{"6.1.5",  1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  1,  1,  1,  1,  1},

	/* wwe10c */
	{"6.2.3",  1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  1,  1,  1,  1,  1},

	/* wwe10a */
	{"6.0.1",  1,  1,  1,  1,  1,  0,  1,  1,  1,  0,  1,  1,  1,  1,  1},

	/* P0106RD: gt2005, 1024x600*/
	{"6.3.1",  1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  1,  1,  1,  1,  1},

	/* 1806RD*/
	{"7.3.1",  1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  1,  1,  1,  1,  1},

	/* P011CZ-1: ov2655, 1024x600*/
	{"6.3.2",  1,  1,  1,  1,  1,  0,  1,  1,  1,  0,  1,  1,  1,  1,  1},

	/* WWE10B-1.7: ov2655, 1024x576*/
	{"6.1.7",  1,  1,  1,  1,  1,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1},

	/* WWE10B-1.8: gt2005, 1024x576*/
	{"6.1.8",  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},

	/* P0106RD-3.3: gt2005, 1024x576*/
	{"6.3.3",  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},

	/* P011CZ-3.4: ov2655, 1024x576*/
	{"6.3.4",  1,  1,  1,  1,  1,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1},
};

static char * imap_product_hwid(void)
{
	char *s;
	char *p;

	s = strstr(saved_command_line, "hwver=");
	if (!s)
		return NULL;
	p = s + 6;
	return strstr(p, ".") + 1;
}

int imap_product_getid(enum imap_product_id id)
{
	char * hwid = imap_product_hwid();
	int i;
	
	if (!hwid)
		return -1;

	printk("hwid is %s\n", hwid);
	for(i = 0; i < ARRAY_SIZE(imap_product_data); i++)
	  if(!strncmp(imap_product_data[i].bid, hwid,
			 strlen(imap_product_data[i].bid)))
		break;

	if(i == ARRAY_SIZE(imap_product_data))
	{
		printk(KERN_ERR "No device id found in table\n");
		return -1;
	}

	switch(id)
	{
		case IMAP_PRODUCT_SDIO:
			return imap_product_data[i].sdio;
		case IMAP_PRODUCT_NAND:
			return imap_product_data[i].nand;
		case IMAP_PRODUCT_PHONE:
			return imap_product_data[i].phone;
		case IMAP_PRODUCT_RF:
			return imap_product_data[i].rf;
		case IMAP_PRODUCT_GSENSOR:
			return imap_product_data[i].gsensor;
		case IMAP_PRODUCT_CAMERA:
			return imap_product_data[i].camera;
		case IMAP_PRODUCT_HDMI:
			return imap_product_data[i].hdmi;
		case IMAP_PRODUCT_ADS:
			return imap_product_data[i].ads;
		case IMAP_PRODUCT_GPIOKEYS:
			return imap_product_data[i].gpio_keys;
		case IMAP_PRODUCT_LCD:
			return imap_product_data[i].lcd;
		case IMAP_PRODUCT_AUDIO:
			return imap_product_data[i].audio;
		case IMAP_PRODUCT_NOR:
			return imap_product_data[i].nor;
		case IMAP_PRODUCT_WIFI:
			return imap_product_data[i].wifi;
		case IMAP_PRODUCT_TP:
			return imap_product_data[i].tp;
		case IMAP_PRODUCT_BATT:
			return imap_product_data[i].batt;
		default:
			printk(KERN_ERR "Attemping to get a device id which do not exist.\n");
			return -1;
	}

	/* never returns zero */
	return 0;
}

EXPORT_SYMBOL_GPL(imap_product_getid);
