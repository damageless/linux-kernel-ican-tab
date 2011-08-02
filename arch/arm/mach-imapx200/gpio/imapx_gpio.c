/*
 * imapx_gpio.c
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
 * Main file of IMAPX platform GPIO layer.
 *
 * Author:
 *	Sololz<sololz.luo@gmail.com>.
 *
 * Revision History:
 * 1.0  04/11/2011 Sololz.
 * 	Create this file.
 * 1.1	04/21/2011 Sololz.
 * 	Add group bat operation support for setcfg, setpin, pull.
 */

#include "imapx_gpio.h"

/* ############################################################################## */

/* Map internal data. */
static const struct imapx_gpio_name_map 
imapx_gpio_name_all_maps[IG_BASE_COUNT] = {
	[IG_BASE_GET_INDEX(IG_BASE_A)] = {	/* GPA */
		.group = IG_BASE_A,
		.group_name_mark = 'A',
	}, [IG_BASE_GET_INDEX(IG_BASE_B)] = {	/* GPB */
		.group = IG_BASE_B,
		.group_name_mark = 'B',
	}, [IG_BASE_GET_INDEX(IG_BASE_C)] = {	/* GPC */
		.group = IG_BASE_C,
		.group_name_mark = 'C',
	}, [IG_BASE_GET_INDEX(IG_BASE_D)] = {	/* GPD */
		.group = IG_BASE_D,
		.group_name_mark = 'D',
	}, [IG_BASE_GET_INDEX(IG_BASE_E)] = {	/* GPE */
		.group = IG_BASE_E,
		.group_name_mark = 'E',
	}, [IG_BASE_GET_INDEX(IG_BASE_F)] = {	/* GPF */
		.group = IG_BASE_F,
		.group_name_mark = 'F',
	}, [IG_BASE_GET_INDEX(IG_BASE_G)] = {	/* GPG */
		.group = IG_BASE_G,
		.group_name_mark = 'G',
	}, [IG_BASE_GET_INDEX(IG_BASE_H)] = {	/* GPH */
		.group = IG_BASE_H,
		.group_name_mark = 'H',
	}, [IG_BASE_GET_INDEX(IG_BASE_I)] = {	/* GPI */
		.group = IG_BASE_I,
		.group_name_mark = 'I',
	}, [IG_BASE_GET_INDEX(IG_BASE_J)] = {	/* GPJ */
		.group = IG_BASE_J,
		.group_name_mark = 'J',
	}, [IG_BASE_GET_INDEX(IG_BASE_K)] = {	/* GPK */
		.group = IG_BASE_K,
		.group_name_mark = 'K',
	}, [IG_BASE_GET_INDEX(IG_BASE_L)] = {	/* GPL */
		.group = IG_BASE_L,
		.group_name_mark = 'L',
	}, [IG_BASE_GET_INDEX(IG_BASE_M)] = {	/* GPM */
		.group = IG_BASE_M,
		.group_name_mark = 'M',
	}, [IG_BASE_GET_INDEX(IG_BASE_N)] = {	/* GPN */
		.group = IG_BASE_N,
		.group_name_mark = 'N',
	}, [IG_BASE_GET_INDEX(IG_BASE_O)] = {	/* GPO */
		.group = IG_BASE_O,
		.group_name_mark = 'O',
	}, [IG_BASE_GET_INDEX(IG_BASE_P)] = {	/* GPP */
		.group = IG_BASE_P,
		.group_name_mark = 'P',
	}, [IG_BASE_GET_INDEX(IG_BASE_Q)] = {	/* GPQ */
		.group = IG_BASE_Q,
		.group_name_mark = 'Q',
	}, [IG_BASE_GET_INDEX(IG_BASE_R)] = {	/* GPR */
		.group = IG_BASE_R,
		.group_name_mark = 'R',
	},
};
#define IMAPX_GPIO_SPECIAL_BITS		(IMAPX_OPOSITE_BIT)
#define IMAPX_GPIO_GET_GROUP(gpio)	\
	imapx_gpio_name_all_maps[(gpio & 0x00ff0000) >> 16].group
#define IMAPX_GPIO_GET_ID(gpio)		(gpio & 0x0000000f)
#define IMAPX_GPIO_GROUP_ERROR(gpio)	\
	unlikely(((gpio & ~(IMAPX_GPIO_SPECIAL_BITS)) >> 16) >= IG_BASE_COUNT)

/* ############################################################################## */

/*
 * FUNCTION
 * __imapx_name_to_gpio()
 *
 * Convert a string gpio name to correspond gpio number.
 */
unsigned int __imapx_name_to_gpio(const char *gpio_name)
{
	unsigned int gpio = 0;
	unsigned int len = 0;
	unsigned int id = 0;
	int i = 0;
	unsigned int id_ok = 0;

	if (unlikely(gpio_name == NULL)) {
		IG_PANIC("GPIO name is NULL!");
		return IMAPX_GPIO_ERROR;
	}
	len = strlen(gpio_name);
	/* The max string length is 6, min length is 4, etc, "GPA1", "GPE15#" */
	if (unlikely((len < 4) || (len > 6))) {
		IG_PANIC("Error string, %s", gpio_name);
		return IMAPX_GPIO_ERROR;
	}
	/* "GPE123" is invalid. */
	if (len == 6) {
		if (unlikely(gpio_name[5] != '#')) {
			IG_PANIC("Error string, %s", gpio_name);
			return IMAPX_GPIO_ERROR;
		}
	}
	/* Check GPIO name validation. */
	if (unlikely((gpio_name[0] != 'G') || (gpio_name[1] != 'P'))) {
		IG_PANIC("Error string, %s", gpio_name);
		return IMAPX_GPIO_ERROR;
	}

	/* Search and calculate group number. */
	while (i < sizeof(imapx_gpio_name_all_maps) / 
			sizeof(imapx_gpio_name_all_maps[0])) {
		if (gpio_name[2] == imapx_gpio_name_all_maps[i].group_name_mark) {
			break;
		}
		i++;
	}
	if (unlikely(i == sizeof(imapx_gpio_name_all_maps) / 
				sizeof(imapx_gpio_name_all_maps[0]))) {
		IG_PANIC("Error string, %s", gpio_name);
		return IMAPX_GPIO_ERROR;
	}
	gpio |= ((IG_BASE_GET_INDEX(imapx_gpio_name_all_maps[i].group)) << 16);

	/* Calculate id. */
	for (i = 0; i < len - 3; i++) {
		if (isdigit(gpio_name[i + 3])) {
			id = (id * 10) + (gpio_name[i + 3] - '0');
			id_ok = 1;
		} else if (gpio_name[i + 3] == '#') {
			/* If first id number not get, or '#' appears during string, 
			 * etc, "GPE#12", "GPE1#2", not allowed. */
			if (unlikely((!id_ok) || (i != len - 4))) {
				IG_PANIC("Error string, %s", gpio_name);
				return IMAPX_GPIO_ERROR;
			}
			gpio |= IMAPX_OPOSITE_BIT;
		} else {
			IG_PANIC("Error string, %s", gpio_name);
			return IMAPX_GPIO_ERROR;
		}
	}
	gpio |= id;

	return gpio;
}
EXPORT_SYMBOL(__imapx_name_to_gpio);

/*
 * FUNCTION
 * imapx_gpio_setslp()
 *
 * Set gpio mode.
 */
unsigned int imapx_gpio_setslp(unsigned int gpio, unsigned int sleep)
{
	if (IMAPX_GPIO_GROUP_ERROR(gpio)) {
		IG_PANIC("GPIO error, 0x%08x.", gpio);
		return IG_BASE_DATA_ERROR;
	}

	return ig_base_setslp(IMAPX_GPIO_GET_GROUP(gpio),
			IMAPX_GPIO_GET_ID(gpio), sleep);
}
EXPORT_SYMBOL(imapx_gpio_setslp);

/*
 * FUNCTION
 * imapx_gpio_is_sleep()
 *
 * Check whether a gpio is set to be sleep mode.
 */
unsigned int imapx_gpio_is_sleep(unsigned int gpio)
{
	if (IMAPX_GPIO_GROUP_ERROR(gpio)) {
		IG_PANIC("GPIO error, 0x%08x.", gpio);
		return IG_BASE_CFG_ERROR;
	}

	return ig_base_is_sleep(IMAPX_GPIO_GET_GROUP(gpio),
			IMAPX_GPIO_GET_ID(gpio));
}
EXPORT_SYMBOL(imapx_gpio_is_sleep);

/*
 * FUNCTION
 * imapx_gpio_setcfg()
 *
 * Set gpio mode to be input/output/other...
 */
unsigned int imapx_gpio_setcfg(unsigned int gpio, unsigned int cfg, 
		unsigned int sleep)
{
	int i = 0;
	unsigned int group = 0;
	unsigned int count = 0;
	unsigned int ret = 0;

	if (IMAPX_GPIO_GROUP_ERROR(gpio)) {
		IG_PANIC("GPIO error, 0x%08x.", gpio);
		return IG_BASE_CFG_ERROR;
	}

	group = IMAPX_GPIO_GET_GROUP(gpio);
	count = IG_BASE_GET_COUNT(group);

	/* Process whole group. */
	if (gpio & IMAPX_ALL_IN_GROUP) {
		for (i = 0; i < count; i++) {
			if (sleep == IG_NMSL) {
				ret = ig_base_setcfg(group, i, cfg, IG_BASE_NORMAL);
				ret = ig_base_setcfg(group, i, cfg, IG_BASE_SLEEP);
			} else {
				ret = ig_base_setcfg(group, i, cfg, sleep);
			}
		}
		return ret;
	}

	/* Process range group. */
	if (gpio & IMAPX_RANGE_GROUP) {
		unsigned int start = 0;
		unsigned int end = 0;
		/* Check start and end validation. */
		start = IMAPX_GET_RANGE_START(gpio);
		end = IMAPX_GET_RANGE_END(gpio);
		if (unlikely((start > end) || (end >= count))) {
			IG_PANIC("GPIO range start or end error, start %d, end %d!", 
					start, end);
			return IG_BASE_CFG_ERROR;
		}
		for (i = start; i <= end; i++) {
			if (sleep == IG_NMSL) {
				ret = ig_base_setcfg(group, i, cfg, IG_BASE_NORMAL);
				ret = ig_base_setcfg(group, i, cfg, IG_BASE_SLEEP);
			} else {
				ret = ig_base_setcfg(group, i, cfg, sleep);
			}
		}
		return ret;
	}

	/* Single set config. */
	if (sleep == IG_NMSL) {
		ret = ig_base_setcfg(group, IMAPX_GPIO_GET_ID(gpio), 
				cfg, IG_BASE_NORMAL);
		ret = ig_base_setcfg(group, IMAPX_GPIO_GET_ID(gpio), 
				cfg, IG_BASE_SLEEP);
	} else {
		ret = ig_base_setcfg(group, IMAPX_GPIO_GET_ID(gpio), 
				cfg, sleep);
	}

	return ret;
}
EXPORT_SYMBOL(imapx_gpio_setcfg);

/*
 * FUNCTION
 * imapx_gpio_getcfg()
 *
 * Get gpio cfg from correspond device gpio.
 */
unsigned int imapx_gpio_getcfg(unsigned int gpio, unsigned int sleep)
{
	if (IMAPX_GPIO_GROUP_ERROR(gpio)) {
		IG_PANIC("GPIO error, 0x%08x.", gpio);
		return IG_BASE_CFG_ERROR;
	}

	return ig_base_getcfg(IMAPX_GPIO_GET_GROUP(gpio),
			IMAPX_GPIO_GET_ID(gpio), sleep);
}
EXPORT_SYMBOL(imapx_gpio_getcfg);

/*
 * FUNCTION
 * imapx_gpio_setpin()
 *
 * Write data to a device correspond gpio.
 */
unsigned int imapx_gpio_setpin(unsigned int gpio, unsigned int data,
		unsigned int sleep)
{
	int i = 0;
	unsigned int group = 0;
	unsigned int count = 0;
	unsigned int ret = 0;

	if (IMAPX_GPIO_GROUP_ERROR(gpio)) {
		IG_PANIC("GPIO error, 0x%08x.", gpio);
		return IG_BASE_DATA_ERROR;
	}

	if (gpio & IMAPX_OPOSITE_BIT) {
		data = (~data) & 1;
	}

	group = IMAPX_GPIO_GET_GROUP(gpio);
	count = IG_BASE_GET_COUNT(group);

	/* Process whole group. */
	if (gpio & IMAPX_ALL_IN_GROUP) {
		for (i = 0; i < count; i++) {
			if (sleep == IG_NMSL) {
				ret = ig_base_setpin(group, i, data, IG_BASE_NORMAL);
				ret = ig_base_setpin(group, i, data, IG_BASE_SLEEP);
			} else {
				ret = ig_base_setpin(group, i, data, sleep);
			}
		}
		return ret;
	}

	/* Process range group. */
	if (gpio & IMAPX_RANGE_GROUP) {
		unsigned int start = 0;
		unsigned int end = 0;
		/* Check start and end validation. */
		start = IMAPX_GET_RANGE_START(gpio);
		end = IMAPX_GET_RANGE_END(gpio);
		if (unlikely((start > end) || (end >= count))) {
			IG_PANIC("GPIO range start or end error, start %d, end %d!", 
					start, end);
			return IG_BASE_CFG_ERROR;
		}
		for (i = start; i <= end; i++) {
			if (sleep == IG_NMSL) {
				ret = ig_base_setpin(group, i, data, IG_BASE_NORMAL);
				ret = ig_base_setpin(group, i, data, IG_BASE_SLEEP);
			} else {
				ret = ig_base_setpin(group, i, data, sleep);
			}
		}
		return ret;
	}

	/* Process single gpio. */
	if (sleep == IG_NMSL) {
		ret = ig_base_setpin(group, IMAPX_GPIO_GET_ID(gpio), 
				data, IG_BASE_NORMAL);
		ret = ig_base_setpin(group, IMAPX_GPIO_GET_ID(gpio), 
				data, IG_BASE_SLEEP);
	} else {
		ret = ig_base_setpin(group, IMAPX_GPIO_GET_ID(gpio), 
				data, sleep);
	}

	return ret;
}
EXPORT_SYMBOL(imapx_gpio_setpin);

/*
 * FUNCTION
 * imapx_gpio_getpin()
 *
 * Read a device correspond gpio.
 */
unsigned int imapx_gpio_getpin(unsigned int gpio, unsigned int sleep)
{
	unsigned int ret = 0;

	if (IMAPX_GPIO_GROUP_ERROR(gpio)) {
		IG_PANIC("GPIO error, 0x%08x.", gpio);
		return IG_BASE_DATA_ERROR;
	}

	ret = ig_base_getpin(IMAPX_GPIO_GET_GROUP(gpio),
			IMAPX_GPIO_GET_ID(gpio), sleep);
	if ((ret != IG_BASE_DATA_ERROR) && 
			(gpio & IMAPX_OPOSITE_BIT)) {
		ret = (~ret) & 1;
	}

	return ret;
}
EXPORT_SYMBOL(imapx_gpio_getpin);

/*
 * FUNCTION
 * imapx_gpio_pull()
 *
 * Directly pull up/down a device correspond gpio.
 */
void imapx_gpio_pull(unsigned int gpio, unsigned int en,
		unsigned int sleep)
{
	int i = 0;
	unsigned int group = 0;
	unsigned int count = 0;

	if (IMAPX_GPIO_GROUP_ERROR(gpio)) {
		IG_PANIC("GPIO error, 0x%08x.", gpio);
		return;
	}

	group = IMAPX_GPIO_GET_GROUP(gpio);
	count = IG_BASE_GET_COUNT(group);

	/* Process whole group. */
	if (gpio & IMAPX_ALL_IN_GROUP) {
		for (i = 0; i < count; i++) {
			if (sleep == IG_NMSL) {
				ig_base_pull(group, i, en, IG_BASE_NORMAL);
				ig_base_pull(group, i, en, IG_BASE_SLEEP);
			} else {
				ig_base_pull(group, i, en, sleep);
			}
		}
		return;
	}

	/* Process range group. */
	if (gpio & IMAPX_RANGE_GROUP) {
		unsigned int start = 0;
		unsigned int end = 0;
		/* Check start and end validation. */
		start = IMAPX_GET_RANGE_START(gpio);
		end = IMAPX_GET_RANGE_END(gpio);
		if (unlikely((start > end) || (end >= count))) {
			IG_PANIC("GPIO range start or end error, start %d, end %d!", 
					start, end);
			return;
		}
		for (i = start; i <= end; i++) {
			if (sleep == IG_NMSL) {
				ig_base_pull(group, i, en, IG_BASE_NORMAL);
				ig_base_pull(group, i, en, IG_BASE_SLEEP);
			} else {
				ig_base_pull(group, i, en, sleep);
			}
		}
		return;
	}

	/* Process single gpio. */
	if (sleep == IG_NMSL) {
		ig_base_pull(group, IMAPX_GPIO_GET_ID(gpio), 
				en, IG_BASE_NORMAL);
		ig_base_pull(group, IMAPX_GPIO_GET_ID(gpio), 
				en, IG_BASE_SLEEP);
	} else {
		ig_base_pull(group, IMAPX_GPIO_GET_ID(gpio), 
				en, sleep);
	}
}
EXPORT_SYMBOL(imapx_gpio_pull);

/*
 * FUNCTION
 * imapx_gpio_getpull()
 *
 * Get GPIO PUD status.
 */
unsigned int imapx_gpio_getpull(unsigned int gpio, unsigned int sleep)
{
	if (IMAPX_GPIO_GROUP_ERROR(gpio)) {
		IG_PANIC("GPIO error, 0x%08x.", gpio);
		return IG_BASE_DATA_ERROR;
	}

	return ig_base_getpull(IMAPX_GPIO_GET_GROUP(gpio),
			IMAPX_GPIO_GET_ID(gpio), sleep);
}
EXPORT_SYMBOL(imapx_gpio_getpull);

/*
 * FUNCTION
 * imapx_gpio_to_irq()
 *
 * Get gpio correspond irq number.
 */
int imapx_gpio_to_irq(unsigned int gpio)
{
	if (IMAPX_GPIO_GROUP_ERROR(gpio)) {
		IG_PANIC("GPIO error, 0x%08x.", gpio);
		return IG_BASE_DATA_ERROR;
	}

	return ig_base_get_irq(IMAPX_GPIO_GET_GROUP(gpio),
			IMAPX_GPIO_GET_ID(gpio));
}

/*
 * FUNCTION
 * imapx_gpio_setirq()
 *
 * Set external irq or group.
 */
unsigned int imapx_gpio_setirq(unsigned int gpio, unsigned int flt, 
		unsigned int mode, unsigned int en)
{
	if (IMAPX_GPIO_GROUP_ERROR(gpio)) {
		IG_PANIC("GPIO error, 0x%08x.", gpio);
		return IG_BASE_DATA_ERROR;
	}

	return ig_base_setirq(IMAPX_GPIO_GET_GROUP(gpio),
			IMAPX_GPIO_GET_ID(gpio), flt, mode, en);
}
EXPORT_SYMBOL(imapx_gpio_setirq);

/*
 * FUNCTION
 * imapx_gpio_is_pending()
 *
 * Check irq pending status.
 */
unsigned int imapx_gpio_is_pending(unsigned int gpio, unsigned int clear)
{
	if (IMAPX_GPIO_GROUP_ERROR(gpio)) {
		IG_PANIC("GPIO error, 0x%08x.", gpio);
		return IG_BASE_DATA_ERROR;
	}

	return ig_base_is_pending(IMAPX_GPIO_GET_GROUP(gpio),
			IMAPX_GPIO_GET_ID(gpio), clear);
}
EXPORT_SYMBOL(imapx_gpio_is_pending);

/*
 * FUNCTION
 * imapx_gpio_clear_pend()
 *
 * Clear gpio corerspond eint or eintg pending bit.
 */
void imapx_gpio_clear_pend(unsigned int gpio)
{
	if (IMAPX_GPIO_GROUP_ERROR(gpio)) {
		IG_PANIC("GPIO error, 0x%08x.", gpio);
		return;
	}

	ig_base_clear_pend(IMAPX_GPIO_GET_GROUP(gpio),
			IMAPX_GPIO_GET_ID(gpio));
}
EXPORT_SYMBOL(imapx_gpio_clear_pend);

/*
 * FUNCTION
 * imapx_gpio_unmask_irq()
 *
 * Unmask gpio correspond eint or eintg.
 */
void imapx_gpio_unmask_irq(unsigned int gpio)
{
	if (IMAPX_GPIO_GROUP_ERROR(gpio)) {
		IG_PANIC("GPIO error, 0x%08x.", gpio);
		return;
	}

	ig_base_unmask_irq(IMAPX_GPIO_GET_GROUP(gpio),
			IMAPX_GPIO_GET_ID(gpio));
}
EXPORT_SYMBOL(imapx_gpio_unmask_irq);

/*
 * FUNCTION
 * imapx_gpio_mask_irq()
 *
 * Mask gpio correspond eint or eintg.
 */
void imapx_gpio_mask_irq(unsigned int gpio)
{
	if (IMAPX_GPIO_GROUP_ERROR(gpio)) {
		IG_PANIC("GPIO error, 0x%08x.", gpio);
		return;
	}

	ig_base_mask_irq(IMAPX_GPIO_GET_GROUP(gpio),
			IMAPX_GPIO_GET_ID(gpio));
}
EXPORT_SYMBOL(imapx_gpio_mask_irq);
