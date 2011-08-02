/*
 * gpio_base_private.h
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
 * A part of IMAPX platform GPIO base layer.
 *
 * Author:
 *	Sololz<sololz.luo@gmail.com>.
 *
 * Revision History:
 * 1.0  04/11/2011 Sololz.
 * 	Create this file.
 */

#if !defined(__GPIO_BASE_PRIVATE_H__)
#define __GPIO_BASE_PRIVATE_H__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>

#include <asm/io.h>
#include <asm/irq.h>

#include <mach/imapx_base_reg.h>
#include <mach/imapx_gpio.h>
#include <mach/imapx_sysmgr.h>
#include <mach/imapx_gpio_api.h>

#include "gpio_base.h"

/* ############################################################################## */

/* FEATURE BITS DEFINITIONS. */
/* On IMAPX200 platform input and output mode config is supported
 * by almost all gpios except GPG(EINT). */
#define IG_BASE_FEATURE_INPUT		(0x00000001)
#define IG_BASE_FEATURE_OUTPUT		(0x00000002)
#define IG_BASE_FEATURE_CTRL0		(0x00000004)
#define IG_BASE_FEATURE_CTRL1		(0x00000008)
#define IG_BASE_FEATURE_OPOSITE		(0x00000010)
#define IG_BASE_FEATURE_PULL_UP		(0x00000020)
#define IG_BASE_FEATURE_PULL_DOWN	(0x00000040)
/* ... */
#define IG_BASE_FEATURE_EINTG_NUM 	(0x0f800000)
#define IG_BASE_FEATURE_EINTG		(0xf0000000)
/* Commmon features of gpio, actually, only input and output is 
 * common features. In fact, GPG is unique. */
#define IG_BASE_FEATURE_COMMON		\
	(IG_BASE_FEATURE_INPUT | IG_BASE_FEATURE_OUTPUT)

/* Bit shift. */
#define IG_BASE_FEATURE_BS_INPUT	(0)
#define IG_BASE_FEATURE_BS_OUTPUT	(1)
#define IG_BASE_FEATURE_BS_CTRL0	(2)
#define IG_BASE_FEATURE_BS_CTRL1	(3)
#define IG_BASE_FEATURE_BS_OPOSITE	(4)
#define IG_BASE_FEATURE_BS_PULL_UP	(5)
#define IG_BASE_FEATURE_BS_PULL_DOWN	(6)
/* ... */
#define IG_BASE_FEATURE_BS_EINTG_NUM	(23)
#define IG_BASE_FEATURE_BS_EINTG	(28)

/* 
 * External interrupt group, in IMAPX200 platform, the external
 * interrupt group index is named from 1 not 0. So, 0 can be an
 * error group number. 
 */
#define IG_BASE_FEATURE_EINTG1		(0x10000000)
#define IG_BASE_FEATURE_EINTG2		(0x20000000)
#define IG_BASE_FEATURE_EINTG3		(0x30000000)
#define IG_BASE_FEATURE_EINTG4		(0x40000000)
#define IG_BASE_FEATURE_EINTG5		(0x50000000)
#define IG_BASE_FEATURE_EINTG6		(0x60000000)
#define IG_BASE_EINTG_COUNT		(6)

/* FEATURE OPERATIONS. */
#define IG_BASE_GPIO_INPUT_ABLE(feature)	\
	(feature & IG_BASE_FEATURE_INPUT)
#define IG_BASE_GPIO_OUTPUT_ABLE(feature)	\
	(feature & IG_BASE_FEATURE_OUTPUT)
#define IG_BASE_GPIO_CTRL0_ABLE(feature)	\
	(feature & IG_BASE_FEATURE_CTRL0)
#define IG_BASE_GPIO_CTRL1_ABLE(feature)	\
	(feature & IG_BASE_FEATURE_CTRL1)
#define IG_BASE_GPIO_IS_OPOSITE(feature)	\
	(feature & IG_BASE_FEATURE_OPOSITE)
/* Actually these two macros is not used because any gpio can be 
 * pull up or down. */
#define IG_BASE_GPIO_PULL_UP_ABLE(feature)	\
	(feature & IG_BASE_FEATURE_PULL_UP)
#define IG_BASE_GPIO_PULL_DOWN_ABLE(feature)	\
	(feature & IG_BASE_FEATURE_PULL_DOWN)
/* External interrupt group related ops. */
#define IG_BASE_GPIO_GET_EINTG(feature)		\
	(feature & IG_BASE_FEATURE_EINTG)
#define IG_BASE_GPIO_GET_EINTG_ID(feature)	\
	(((feature & IG_BASE_FEATURE_EINTG) >> IG_BASE_FEATURE_BS_EINTG) - 1)
#define IG_BASE_GPIO_GET_EINTG_NUM(feature)	\
	((feature & IG_BASE_FEATURE_EINTG_NUM) >> IG_BASE_FEATURE_BS_EINTG_NUM)
#define IG_BASE_GPIO_SET_EINTG_NUM(num)		\
	(num << IG_BASE_FEATURE_BS_EINTG_NUM)
struct ig_base_gpio_spec {
	/* 
	 * I just use a 32bit variable to stores all kinds of gpio
	 * features. So I have to define a lot of macros to support
	 * basic operation of the feature variable and these macros
	 * might seems to be shit. But it really greatly reduces the
	 * memory cost. PS: 117 gpios total.
	 */
	unsigned int feature;
};

#define IG_BASE_IRQ_FILTER_ENABLE	(0x00000080)

/* Group structure of gpio, which is designed to contain all informations. */
struct ig_base_group {
	const struct ig_base_gpio_spec *spec;
	unsigned int count;
	void __iomem *dat;
	void __iomem *con;
	void __iomem *pud;
	spinlock_t lock;
};

/* Sleep able group map structure. */
struct ig_base_sleep_map {
	void __iomem *ctrl;
	void __iomem *dat;
	void __iomem *con;
	void __iomem *pud;
};

/* External interrupt group map structure. */
struct ig_base_eintg_map {
	void __iomem *mask;
	void __iomem *pend;
};

#endif	/* __GPIO_BASE_PRIVATE_H__ */
