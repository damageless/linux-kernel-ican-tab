/************************************************************************************
 * soops.h
 *
 * Copyright (c) 2009~2014 ShangHai Infotm Ltd all rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Head file of soops char devices driver.
 *
 * Sololz <sololz.luo@gmail.com>.
 *
 * Revision History:
 * 1.0	2010/12/28 Sololz
 * 	Create this file.
 ***********************************************************************************/

#if !defined(__SOOPS_H__)
#define __SOOPS_H__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>

#define SOOPS_DEV_NAME	"soops"

/* Debug print macros. */
#if defined(CONFIG_SOOPS_DEBUG)
#define soops_debug(debug, ...)	do {printk(KERN_DEBUG "[SOOPS DEBUG] " debug, ##__VA_ARGS__);} while (0)
#else
#define soops_debug(debug, ...) do {} while (0)
#endif
#define soops_error(error, ...)	do {printk(KERN_ERR "[SOOPS ERROR] %s %d " error, __FUNCTION__, __LINE__, ##__VA_ARGS__);} while (0)
#define soops_alert(alert, ...)	do {printk(KERN_ALERT "[SOOPS ALERT] " alert, ##__VA_ARGS__)} while (0)

/* IO control commands. */
#define SOOPS_IOCMD_MAGIC		'S'
#define SOOPS_IOCMD_GET_CAPS_STATUS	_IOR(SOOPS_IOCMD_MAGIC, 1, int)
#if defined(CONFIG_INPUT_KEYBOARD_SPANISH)
#define SOOPS_IOCMD_GET_SPANKEY_STATUS	_IOR(SOOPS_IOCMD_MAGIC, 2, int)
#define SOOPS_IOCMD_CLEAR_SPANKEY_STATUS _IO(SOOPS_IOCMD_MAGIC, 5)
#endif
#define SOOPS_IOCMD_TEST		_IO(SOOPS_IOCMD_MAGIC, 4)
#if defined(CONFIG_KEYBOARD_MATRIX)
#define SOOPS_IOCMD_ENABLE_MATRIX_KEYBOARD 		_IO(SOOPS_IOCMD_MAGIC, 3)
#endif
#define SOOPS_IOCMD_GET_SHIFTKEY_STATUS	 _IOR(SOOPS_IOCMD_MAGIC, 6, int)
#define SOOPS_IOCMD_GET_ALTKEY_STATUS  _IOR(SOOPS_IOCMD_MAGIC, 7, int)
#define SOOPS_IOCMD_MAX_NUM		7

#endif	/* __SOOPS_H__ */

