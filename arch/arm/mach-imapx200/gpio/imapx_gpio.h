/*
 * imapx_gpio.h
 *
 * Copyright (c) 2009~2014 ShangHai Infotm Ltd all rights reserved.
 *
 * Use of Infotm's code is governed by terms and conditions
 * stated in the accompanying licensing statement.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * A part of IMAPX platform GPIO device layer. Internal head file
 * of device layer.
 *
 * Author:
 *	Sololz<sololz.luo@gmail.com>.
 *
 * Revision History:
 * 1.0  04/11/2011 Sololz.
 * 	Create this file.
 */

#if !defined(__IMAPX_GPIO_H__)
#define __IMAPX_GPIO_H__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ctype.h>

#include <mach/imapx_gpio_api.h>

#include "base/gpio_base.h"

/* ############################################################################## */

/* GPIO group name map with base layer macro definitions. */
struct imapx_gpio_name_map {
	unsigned int group;
	char group_name_mark;
};

/* ############################################################################## */

#endif	/* __IMAPX_GPIO_H__ */
