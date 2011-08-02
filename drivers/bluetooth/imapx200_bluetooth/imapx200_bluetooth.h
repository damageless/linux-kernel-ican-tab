/***************************************************************************** 
 * imapx200_bluetooth.h
 * 
 * Copyright (c) 2009~2014 ShangHai Infotm Ltd all rights reserved. 
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * Description:
 *	Commmon head file for IMAPX200 bluetooth driver.
 *
 * Author:
 *	Sololz <sololz.luo@gmail.com>.
 *      
 * Revision History: 
 * ­­­­­­­­­­­­­­­­­ 
 * 1.0  2011/01/06 Sololz
 * 	Create this file.
 ******************************************************************************/

#if !defined(__IMAPX200_BLUETOOTH_H__)
#define __IMAPX200_BLUETOOTH_H__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/rfkill.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>

/* ############################################################################ */

/* Debug macros include debug alert error. */
#ifdef CONFIG_IMAPX200_BLUETOOTH_DEBUG
#define imxbt_debug(debug, ...)	\
	printk(KERN_DEBUG "%s line %d: " debug, __func__, __LINE__, ##__VA_ARGS__)
#else
#define imxbt_debug(debug, ...) do {} while (0)
#endif
#define imxbt_alert(alert, ...)	\
	printk(KERN_ALERT "%s line %d: " alert, __func__, __LINE__, ##__VA_ARGS__)
#define imxbt_error(error, ...)	\
	printk(KERN_ERR "%s line %d: " error, __func__, __LINE__, ##__VA_ARGS__)

#endif	/* __IMAPX200_BLUETOOTH_H__ */
