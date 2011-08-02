/*
 * gpio_base.h
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

#if !defined(__GPIO_BASE_H__)
#define __GPIO_BASE_H__

/* ############################################################################## */

/* Panic. */
#if defined(CONFIG_IG_PANIC_ENABLE)
#define IG_PANIC		panic
#else
#define IG_PANIC(fmt, ...)	printk(KERN_ALERT fmt, ##__VA_ARGS__)
#endif
/* Debug. */
#if defined(CONFIG_IG_DEBUG_ENABLE)
#define IG_DEBUG(fmt, ...)	printk(KERN_ALERT fmt, ##__VA_ARGS__)
#else
#define IG_DEBUG(fmt, ...)	do {} while (0)
#endif

/* ############################################################################## */

/* 
 * Base layer wraps all gpio entries, others should not dirrectly access
 * GPX controll register like "readl(rGPXCON)" etc. The prefix of GPX name
 * means IMAPX GPIO GROUP.
 * About the hex number of gpio group name:
 * [31] - Reserved.
 * [30:29] - GPIO group sleep index.
 * [28] - Mark as current GPIO group is single bit per one or not, etc GPG.
 * [27:24] - Reserved.
 * [23:16] - Index of GPIO group, actually the ASIC number of X.
 * [15:8] - Reserved.
 * [7:0] - GPIO count of current group.
 */
/* Bits definitions. */
#define IG_BASE_GROUP_RSV_31		(0x80000000)
#define IG_BASE_GROUP_SLEEP		(0x60000000)	/* [30:29] */
#define IG_BASE_GROUP_SINGLE		(0x10000000)	/* [28] */
#define IG_BASE_GROUP_RSV_27_24		(0x0f000000)
#define IG_BASE_GROUP_GROUP		(0x00ff0000)	/* [23:16] */
#define IG_BASE_GROUP_RSV_15_8		(0x0000ff00)
#define IG_BASE_GROUP_COUNT		(0x000000ff)	/* [7:0] */
/* Bit shift definitions. */
#define IG_BASE_GROUP_BS_SLEEP		(29)
#define IG_BASE_GROUP_BS_SINGLE		(28)
#define IG_BASE_GROUP_BS_GROUP		(16)
#define IG_BASE_GROUP_BS_COUNT		(0)
/* Sleep groups. */
#define IG_BASE_GROUP_SLEEP_A		(0x20000000)
#define IG_BASE_GROUP_SLEEP_B		(0x40000000)
#define IG_BASE_GROUP_SLEEP_O		(0x60000000)
/* Group and count. */
#define IG_BASE_GROUP_SET_COUNT	
#define IG_BASE_GROUP_SET_GROUP(gid)	(((unsigned int)gid) << 16)
/* GPA */
#define IG_BASE_A	\
	(IG_BASE_GROUP_SET_GROUP('A') | \
	 IG_BASE_GROUP_SET_COUNT(8) | \
	 IG_BASE_GROUP_SLEEP_A)
/* GPB */
#define IG_BASE_B	\
	(IG_BASE_GROUP_SET_GROUP('B') | \
	 IG_BASE_GROUP_SET_COUNT(5) | \
	 IG_BASE_GROUP_SLEEP_B)
/* GPC */
#define IG_BASE_C	\
	(IG_BASE_GROUP_SET_GROUP('C') | \
	 IG_BASE_GROUP_SET_COUNT(8))
/* GPD */
#define IG_BASE_D	\
	(IG_BASE_GROUP_SET_GROUP('D') | \
	 IG_BASE_GROUP_SET_COUNT(5))
/* GPE */
#define IG_BASE_E	\
	(IG_BASE_GROUP_SET_GROUP('E') | \
	 IG_BASE_GROUP_SET_COUNT(16))
/* GPF */
#define IG_BASE_F	\
	(IG_BASE_GROUP_SET_GROUP('F') | \
	 IG_BASE_GROUP_SET_COUNT(10))
/* GPG */
#define IG_BASE_G	\
	(IG_BASE_GROUP_SET_GROUP('G') | \
	 IG_BASE_GROUP_SET_COUNT(6) | \
	 IG_BASE_GROUP_SINGLE)
/* GPH */
#define IG_BASE_H	\
	(IG_BASE_GROUP_SET_GROUP('H') | \
	 IG_BASE_GROUP_SET_COUNT(4))
/* GPI */
#define IG_BASE_I	\
	(IG_BASE_GROUP_SET_GROUP('I') | \
	 IG_BASE_GROUP_SET_COUNT(14))
/* GPJ */
#define IG_BASE_J	\
	(IG_BASE_GROUP_SET_GROUP('J') | \
	 IG_BASE_GROUP_SET_COUNT(9))
/* GPK */
#define IG_BASE_K	\
	(IG_BASE_GROUP_SET_GROUP('K') | \
	 IG_BASE_GROUP_SET_COUNT(16))
/* GPL */
#define IG_BASE_L	\
	(IG_BASE_GROUP_SET_GROUP('L') | \
	 IG_BASE_GROUP_SET_COUNT(13))
/* GPM */
#define IG_BASE_M	\
	(IG_BASE_GROUP_SET_GROUP('M') | \
	 IG_BASE_GROUP_SET_COUNT(16))
/* GPN */
#define IG_BASE_N	\
	(IG_BASE_GROUP_SET_GROUP('N') | \
	 IG_BASE_GROUP_SET_COUNT(13))
/* GPO */
#define IG_BASE_O	\
	(IG_BASE_GROUP_SET_GROUP('O') | \
	 IG_BASE_GROUP_SET_COUNT(16) | \
	 IG_BASE_GROUP_SLEEP_O)
/* GPP */
#define IG_BASE_P	\
	(IG_BASE_GROUP_SET_GROUP('P') | \
	 IG_BASE_GROUP_SET_COUNT(12))
/* GPQ */
#define IG_BASE_Q	\
	(IG_BASE_GROUP_SET_GROUP('Q') | \
	 IG_BASE_GROUP_SET_COUNT(6))
/* GPR */
#define IG_BASE_R	\
	(IG_BASE_GROUP_SET_GROUP('R') | \
	 IG_BASE_GROUP_SET_COUNT(16))

/* IG_BASE_X operations and informations. */
#define IG_BASE_COUNT			(18)
#define IG_BASE_SLEEP_COUNT		(3)
#define IG_BASE_GET_COUNT(group)	(group & IG_BASE_GROUP_COUNT)
#define IG_BASE_GET_GROUP(group)	\
	((group & IG_BASE_GROUP_GROUP) >> IG_BASE_GROUP_BS_GROUP)
#define IG_BASE_GET_INDEX(group)	\
	(IG_BASE_GET_GROUP(group) - IG_BASE_GET_GROUP(IG_BASE_A))
/* IMAPX GPIO group features. */
#define IG_BASE_IS_SINGLE(group)	(group & IG_BASE_GROUP_SINGLE)
#define IG_BASE_CAN_SLEEP(group)	(group & IG_BASE_GROUP_SLEEP)
#define IG_BASE_GET_SLEEP_INDEX(group)	\
	(((group & IG_BASE_GROUP_SLEEP) >> IG_BASE_GROUP_BS_SLEEP) - 1)

/* ############################################################################## */

/**
 * Set gpio mode to be normal or sleep.
 */
unsigned int ig_base_setslp(unsigned int group, unsigned int id, 
		unsigned int sleep);

/**
 * Call this function to check whether gpio is set to be sleep mode.
 *
 * Return:
 * @ If gpio is set to be sleep mode, returns none zeror number.
 * @ If gpio is in normal mode, returns 0.
 */
unsigned int ig_base_is_sleep(unsigned int group, unsigned int id);

/**
 * Set a gpio cfg. Do careful use all following APIs, if an error or unexist 
 * gpio is used, function will directly call panic() to protect kernel from
 * unexpect results.
 *
 * Return:
 * @ IG_BASE_MODE_ERROR - Some errors occured.
 * @ If success, returns the assigned *cfg*.
 */
unsigned int ig_base_setcfg(unsigned int group, unsigned int id, 
		unsigned int cfg, unsigned int sleep);

/**
 * Get *group* and *id* correspond gpio cfg status. The gpio pin might be
 * set to be sleep mode, so, if the current gpio is set to be sleep mode,
 * this API will return the sleep cfg. Else, returns the normal mode cfg.
 *
 * Return:
 * @ If success, return the gpio correspond *cfg*.
 * @ If error, returns IG_BASE_CFG_ERROR.
 */
unsigned int ig_base_getcfg(unsigned int group, unsigned int id,
		unsigned int sleep);

/**
 * Pull up or down a gpio directly. If current gpio is in sleep mode,
 * this function will pull the sleep PUD register.
 */
void ig_base_pull(unsigned int group, unsigned int id, 
		unsigned int en, unsigned int sleep);

/**
 * Get GPIO correspond PUD status. If current gpio is in sleep mode,
 * it will returns the sleep PUD status.
 *
 * Return:
 * @ IG_BASE_DATA_ERROR - Some errors occured.
 * @ If success, returns the pull status.
 */
unsigned int ig_base_getpull(unsigned int group, unsigned int id,
		unsigned int sleep);

/**
 * Read data from assgined gpio. If current gpio is in sleep mode,
 * it will return the sleep DAT status.
 *
 * Return:
 * @ If returns IG_BASE_DATA_ERROR, some errors occured, else the 
 * gpio data 0/1.
 */
unsigned int ig_base_getpin(unsigned int group, unsigned int id, 
		unsigned int sleep);

/**
 * Write data to assigned gpio. If current gpio is in sleep mode,
 * it will set the sleep DAT register.
 *
 * Return:
 * @ IG_BASE_DATA_ERROR - Some errors occured.
 * @ If success, returns the written value.
 */
unsigned int ig_base_setpin(unsigned int group, unsigned int id, 
		unsigned int data, unsigned int sleep);

/* ############################################################################## */

/**
 * Get gpio correspond irq number.
 *
 * Return:
 * @ If success, returns the irq number.
 * @ If some errors, returns -1.
 */
 int ig_base_get_irq(unsigned int group, unsigned int id);

/**
 * Set external interruption generate mode.
 *
 * Return:
 * @ If success, returns the mode set.
 * @ If error, returns IG_BASE_DATA_ERROR.
 */
unsigned int ig_base_setirq(unsigned int group, unsigned int id, 
		unsigned int flt, unsigned int mode, unsigned int en);

/**
 * Check whether GPIO correspond irq is pending. Set the *clear* parameter
 * to auto clear pend bit before function returns.
 *
 * Return:
 * @ 0 - No irq pending status.
 * @ none zero - Get correspond irq pending status.
 */
unsigned int ig_base_is_pending(unsigned int group, unsigned int id,
		unsigned int clear);

/**
 * Clear GPIO correspond irq pending status.
 */
void ig_base_clear_pend(unsigned int group, unsigned int id);

/**
 * Unmask GPIO correspond irq.
 */
void ig_base_unmask_irq(unsigned int group, unsigned int id);

/**
 * Mask GPIO correspond irq.
 */
void ig_base_mask_irq(unsigned int group, unsigned int id);

#endif	/* __GPIO_BASE_H__ */
