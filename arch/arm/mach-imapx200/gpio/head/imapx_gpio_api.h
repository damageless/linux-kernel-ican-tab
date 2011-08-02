/*
 * imapx_gpio_api.h
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
 * Head file to export GPIO program interfaces.
 *
 * Author:
 *	Sololz<sololz.luo@gmail.com>.
 *
 * Revision History:
 * 1.0  04/11/2011 Sololz.
 * 	Create this file.
 */

#define IG_BASE_DATA_ERROR	(0xffffffff)

/* GPIO cfg. */
#define IG_BASE_CFG_INPUT	(0)
#define IG_BASE_CFG_OUTPUT	(1)
#define IG_BASE_CFG_CTRL0	(2)
#define IG_BASE_CFG_CTRL1	(3)
#define IG_BASE_CFG_ERROR	IG_BASE_DATA_ERROR

/* GPIO external interrupt mode. */
#define IG_BASE_EINT_MODE_LL	(0)	/* Low level. */
#define IG_BASE_EINT_MODE_HL	(1)	/* High level. */
#define IG_BASE_EINT_MODE_FET	(2)	/* Falling edge triggled. */
#define IG_BASE_EINT_MODE_RET	(4)	/* Rising edge triggled. */
#define IG_BASE_EINT_MODE_BET	(6)	/* Both edge triggled. */
/* Filter relate macros. */
#define IG_BASE_FILTER_MAX	(0x7f)	/* 7bits all 1. */
#define IG_BASE_FILETER_NONE	(0)
/* Sleep mode. */
#define IG_BASE_NORMAL		(0)
#define IG_BASE_SLEEP		(1)


/* For easy to use */
#define IG_NORMAL		IG_BASE_NORMAL
#define IG_SLEEP		IG_BASE_SLEEP
#define IG_NMSL			(IG_NORMAL + IG_SLEEP + 1)

#define FILTER_MAX		IG_BASE_FILTER_MAX
#define FILTER_NONE		IG_BASE_FILTER_NONE

#define IG_LOW			IG_BASE_EINT_MODE_LL
#define IG_HIGH			IG_BASE_EINT_MODE_HL
#define IG_FALL			IG_BASE_EINT_MODE_FET
#define IG_RSIE			IG_BASE_EINT_MODE_RET
#define IG_BOTH			IG_BASE_EINT_MODE_BET

#define IG_INPUT		IG_BASE_CFG_INPUT
#define IG_OUTPUT		IG_BASE_CFG_OUTPUT
#define IG_CTRL0		IG_BASE_CFG_CTRL0
#define IG_CTRL1		IG_BASE_CFG_CTRL1

/* ############################################################################## */
/* 
 * GPIO number definitions. 
 * [31] - oposite bit.
 * [23:16] - group id.
 * [15:12] - start of group range.
 * [11:8] - end of group range.
 * [5] - range group mark.
 * [4] - all gpio in one group mark.
 * [3:0] - gpio id.
 */
#define IMAPX_OPOSITE_BIT	(0x80000000)
#define IMAPX_ALL_IN_GROUP	(0x00000010)
#define IMAPX_RANGE_GROUP	(0x00000020)

#define IMAPX_GPIO_ERROR	(0xffffffff)
#define IMAPX_NO_GPIO		IMAPX_GPIO_ERROR
/* GPA */
#define IMAPX_GPA0		(0x00000000)
#define IMAPX_GPA1		(0x00000001)
#define IMAPX_GPA2		(0x00000002)
#define IMAPX_GPA3		(0x00000003)
#define IMAPX_GPA4		(0x00000004)
#define IMAPX_GPA5		(0x00000005)
#define IMAPX_GPA6		(0x00000006)
#define IMAPX_GPA7		(0x00000007)
/* GPB */
#define IMAPX_GPB0		(0x00010000)
#define IMAPX_GPB1		(0x00010001)
#define IMAPX_GPB2		(0x00010002)
#define IMAPX_GPB3		(0x00010003)
#define IMAPX_GPB4		(0x00010004)
/* GPC */
#define IMAPX_GPC0		(0x00020000)
#define IMAPX_GPC1		(0x00020001)
#define IMAPX_GPC2		(0x00020002)
#define IMAPX_GPC3		(0x00020003)
#define IMAPX_GPC4		(0x00020004)
#define IMAPX_GPC5		(0x00020005)
#define IMAPX_GPC6		(0x00020006)
#define IMAPX_GPC7		(0x00020007)
/* GPD */
#define IMAPX_GPD0		(0x00030000)
#define IMAPX_GPD1		(0x00030001)
#define IMAPX_GPD2		(0x00030002)
#define IMAPX_GPD3		(0x00030003)
#define IMAPX_GPD4		(0x00030004)
/* GPE */
#define IMAPX_GPE0		(0x00040000)
#define IMAPX_GPE1		(0x00040001)
#define IMAPX_GPE2		(0x00040002)
#define IMAPX_GPE3		(0x00040003)
#define IMAPX_GPE4		(0x00040004)
#define IMAPX_GPE5		(0x00040005)
#define IMAPX_GPE6		(0x00040006)
#define IMAPX_GPE7		(0x00040007)
#define IMAPX_GPE8		(0x00040008)
#define IMAPX_GPE9		(0x00040009)
#define IMAPX_GPE10		(0x0004000a)
#define IMAPX_GPE11		(0x0004000b)
#define IMAPX_GPE12		(0x0004000c)
#define IMAPX_GPE13		(0x0004000d)
#define IMAPX_GPE14		(0x0004000e)
#define IMAPX_GPE15		(0x0004000f)
/* GPF */
#define IMAPX_GPF0		(0x00050000)
#define IMAPX_GPF1		(0x00050001)
#define IMAPX_GPF2		(0x00050002)
#define IMAPX_GPF3		(0x00050003)
#define IMAPX_GPF4		(0x00050004)
#define IMAPX_GPF5		(0x00050005)
#define IMAPX_GPF6		(0x00050006)
#define IMAPX_GPF7		(0x00050007)
#define IMAPX_GPF8		(0x00050008)
#define IMAPX_GPF9		(0x00050009)
/* GPG */
#define IMAPX_GPG0		(0x00060000)
#define IMAPX_GPG1		(0x00060001)
#define IMAPX_GPG2		(0x00060002)
#define IMAPX_GPG3		(0x00060003)
#define IMAPX_GPG4		(0x00060004)
#define IMAPX_GPG5		(0x00060005)
/* GPH */
#define IMAPX_GPH0		(0x00070000)
#define IMAPX_GPH1		(0x00070001)
#define IMAPX_GPH2		(0x00070002)
#define IMAPX_GPH3		(0x00070003)
/* GPI */
#define IMAPX_GPI0		(0x00080000)
#define IMAPX_GPI1		(0x00080001)
#define IMAPX_GPI2		(0x00080002)
#define IMAPX_GPI3		(0x00080003)
#define IMAPX_GPI4		(0x00080004)
#define IMAPX_GPI5		(0x00080005)
#define IMAPX_GPI6		(0x00080006)
#define IMAPX_GPI7		(0x00080007)
#define IMAPX_GPI8		(0x00080008)
#define IMAPX_GPI9		(0x00080009)
#define IMAPX_GPI10		(0x0008000a)
#define IMAPX_GPI11		(0x0008000b)
#define IMAPX_GPI12		(0x0008000c)
#define IMAPX_GPI13		(0x0008000d)
/* GPJ */
#define IMAPX_GPJ0		(0x00090000)
#define IMAPX_GPJ1		(0x00090001)
#define IMAPX_GPJ2		(0x00090002)
#define IMAPX_GPJ3		(0x00090003)
#define IMAPX_GPJ4		(0x00090004)
#define IMAPX_GPJ5		(0x00090005)
#define IMAPX_GPJ6		(0x00090006)
#define IMAPX_GPJ7		(0x00090007)
#define IMAPX_GPJ8		(0x00090008)
/* GPK */
#define IMAPX_GPK0		(0x000a0000)
#define IMAPX_GPK1		(0x000a0001)
#define IMAPX_GPK2		(0x000a0002)
#define IMAPX_GPK3		(0x000a0003)
#define IMAPX_GPK4		(0x000a0004)
#define IMAPX_GPK5		(0x000a0005)
#define IMAPX_GPK6		(0x000a0006)
#define IMAPX_GPK7		(0x000a0007)
#define IMAPX_GPK8		(0x000a0008)
#define IMAPX_GPK9		(0x000a0009)
#define IMAPX_GPK10		(0x000a000a)
#define IMAPX_GPK11		(0x000a000b)
#define IMAPX_GPK12		(0x000a000c)
#define IMAPX_GPK13		(0x000a000d)
#define IMAPX_GPK14		(0x000a000e)
#define IMAPX_GPK15		(0x000a000f)
/* GPL */
#define IMAPX_GPL0		(0x000b0000)
#define IMAPX_GPL1		(0x000b0001)
#define IMAPX_GPL2		(0x000b0002)
#define IMAPX_GPL3		(0x000b0003)
#define IMAPX_GPL4		(0x000b0004)
#define IMAPX_GPL5		(0x000b0005)
#define IMAPX_GPL6		(0x000b0006)
#define IMAPX_GPL7		(0x000b0007)
#define IMAPX_GPL8		(0x000b0008)
#define IMAPX_GPL9		(0x000b0009)
#define IMAPX_GPL10		(0x000b000a)
#define IMAPX_GPL11		(0x000b000b)
#define IMAPX_GPL12		(0x000b000c)
/* GPM */
#define IMAPX_GPM0		(0x000c0000)
#define IMAPX_GPM1		(0x000c0001)
#define IMAPX_GPM2		(0x000c0002)
#define IMAPX_GPM3		(0x000c0003)
#define IMAPX_GPM4		(0x000c0004)
#define IMAPX_GPM5		(0x000c0005)
#define IMAPX_GPM6		(0x000c0006)
#define IMAPX_GPM7		(0x000c0007)
#define IMAPX_GPM8		(0x000c0008)
#define IMAPX_GPM9		(0x000c0009)
#define IMAPX_GPM10		(0x000c000a)
#define IMAPX_GPM11		(0x000c000b)
#define IMAPX_GPM12		(0x000c000c)
#define IMAPX_GPM13		(0x000c000d)
#define IMAPX_GPM14		(0x000c000e)
#define IMAPX_GPM15		(0x000c000f)
/* GPN */
#define IMAPX_GPN0		(0x000d0000)
#define IMAPX_GPN1		(0x000d0001)
#define IMAPX_GPN2		(0x000d0002)
#define IMAPX_GPN3		(0x000d0003)
#define IMAPX_GPN4		(0x000d0004)
#define IMAPX_GPN5		(0x000d0005)
#define IMAPX_GPN6		(0x000d0006)
#define IMAPX_GPN7		(0x000d0007)
#define IMAPX_GPN8		(0x000d0008)
#define IMAPX_GPN9		(0x000d0009)
#define IMAPX_GPN10		(0x000d000a)
#define IMAPX_GPN11		(0x000d000b)
#define IMAPX_GPN12		(0x000d000c)
/* GPO */
#define IMAPX_GPO0		(0x000e0000)
#define IMAPX_GPO1		(0x000e0001)
#define IMAPX_GPO2		(0x000e0002)
#define IMAPX_GPO3		(0x000e0003)
#define IMAPX_GPO4		(0x000e0004)
#define IMAPX_GPO5		(0x000e0005)
#define IMAPX_GPO6		(0x000e0006)
#define IMAPX_GPO7		(0x000e0007)
#define IMAPX_GPO8		(0x000e0008)
#define IMAPX_GPO9		(0x000e0009)
#define IMAPX_GPO10		(0x000e000a)
#define IMAPX_GPO11		(0x000e000b)
#define IMAPX_GPO12		(0x000e000c)
#define IMAPX_GPO13		(0x000e000d)
#define IMAPX_GPO14		(0x000e000e)
#define IMAPX_GPO15		(0x000e000f)
/* GPP */
#define IMAPX_GPP0		(0x000f0000)
#define IMAPX_GPP1		(0x000f0001)
#define IMAPX_GPP2		(0x000f0002)
#define IMAPX_GPP3		(0x000f0003)
#define IMAPX_GPP4		(0x000f0004)
#define IMAPX_GPP5		(0x000f0005)
#define IMAPX_GPP6		(0x000f0006)
#define IMAPX_GPP7		(0x000f0007)
#define IMAPX_GPP8		(0x000f0008)
#define IMAPX_GPP9		(0x000f0009)
#define IMAPX_GPP10		(0x000f000a)
#define IMAPX_GPP11		(0x000f000b)
/* GPQ */
#define IMAPX_GPQ0		(0x00100000)
#define IMAPX_GPQ1		(0x00100001)
#define IMAPX_GPQ2		(0x00100002)
#define IMAPX_GPQ3		(0x00100003)
#define IMAPX_GPQ4		(0x00100004)
#define IMAPX_GPQ5		(0x00100005)
/* GPR */
#define IMAPX_GPR0		(0x00110000)
#define IMAPX_GPR1		(0x00110001)
#define IMAPX_GPR2		(0x00110002)
#define IMAPX_GPR3		(0x00110003)
#define IMAPX_GPR4		(0x00110004)
#define IMAPX_GPR5		(0x00110005)
#define IMAPX_GPR6		(0x00110006)
#define IMAPX_GPR7		(0x00110007)
#define IMAPX_GPR8		(0x00110008)
#define IMAPX_GPR9		(0x00110009)
#define IMAPX_GPR10		(0x0011000a)
#define IMAPX_GPR11		(0x0011000b)
#define IMAPX_GPR12		(0x0011000c)
#define IMAPX_GPR13		(0x0011000d)
#define IMAPX_GPR14		(0x0011000e)
#define IMAPX_GPR15		(0x0011000f)
/* GPIO batch process macros. */
#define IMAPX_GPA(id)		((IMAPX_GPA0 & 0x00ff0000) | id)
#define IMAPX_GPB(id)		((IMAPX_GPB0 & 0x00ff0000) | id)
#define IMAPX_GPC(id)		((IMAPX_GPC0 & 0x00ff0000) | id)
#define IMAPX_GPD(id)		((IMAPX_GPD0 & 0x00ff0000) | id)
#define IMAPX_GPE(id)		((IMAPX_GPE0 & 0x00ff0000) | id)
#define IMAPX_GPF(id)		((IMAPX_GPF0 & 0x00ff0000) | id)
#define IMAPX_GPG(id)		((IMAPX_GPG0 & 0x00ff0000) | id)
#define IMAPX_GPH(id)		((IMAPX_GPH0 & 0x00ff0000) | id)
#define IMAPX_GPI(id)		((IMAPX_GPI0 & 0x00ff0000) | id)
#define IMAPX_GPJ(id)		((IMAPX_GPJ0 & 0x00ff0000) | id)
#define IMAPX_GPK(id)		((IMAPX_GPK0 & 0x00ff0000) | id)
#define IMAPX_GPL(id)		((IMAPX_GPL0 & 0x00ff0000) | id)
#define IMAPX_GPM(id)		((IMAPX_GPM0 & 0x00ff0000) | id)
#define IMAPX_GPN(id)		((IMAPX_GPN0 & 0x00ff0000) | id)
#define IMAPX_GPO(id)		((IMAPX_GPO0 & 0x00ff0000) | id)
#define IMAPX_GPP(id)		((IMAPX_GPP0 & 0x00ff0000) | id)
#define IMAPX_GPQ(id)		((IMAPX_GPQ0 & 0x00ff0000) | id)
#define IMAPX_GPR(id)		((IMAPX_GPR0 & 0x00ff0000) | id)
/* Oposite. */
#define IMAPX_GPA_OP(id)	((IMAPX_GPA0 & 0x00ff0000) | id | IMAPX_OPOSITE_BIT)
#define IMAPX_GPB_OP(id)	((IMAPX_GPB0 & 0x00ff0000) | id | IMAPX_OPOSITE_BIT)
#define IMAPX_GPC_OP(id)	((IMAPX_GPC0 & 0x00ff0000) | id | IMAPX_OPOSITE_BIT)
#define IMAPX_GPD_OP(id)	((IMAPX_GPD0 & 0x00ff0000) | id | IMAPX_OPOSITE_BIT)
#define IMAPX_GPE_OP(id)	((IMAPX_GPE0 & 0x00ff0000) | id | IMAPX_OPOSITE_BIT)
#define IMAPX_GPF_OP(id)	((IMAPX_GPF0 & 0x00ff0000) | id | IMAPX_OPOSITE_BIT)
#define IMAPX_GPG_OP(id)	((IMAPX_GPG0 & 0x00ff0000) | id | IMAPX_OPOSITE_BIT)
#define IMAPX_GPH_OP(id)	((IMAPX_GPH0 & 0x00ff0000) | id | IMAPX_OPOSITE_BIT)
#define IMAPX_GPI_OP(id)	((IMAPX_GPI0 & 0x00ff0000) | id | IMAPX_OPOSITE_BIT)
#define IMAPX_GPJ_OP(id)	((IMAPX_GPJ0 & 0x00ff0000) | id | IMAPX_OPOSITE_BIT)
#define IMAPX_GPK_OP(id)	((IMAPX_GPK0 & 0x00ff0000) | id | IMAPX_OPOSITE_BIT)
#define IMAPX_GPL_OP(id)	((IMAPX_GPL0 & 0x00ff0000) | id | IMAPX_OPOSITE_BIT)
#define IMAPX_GPM_OP(id)	((IMAPX_GPM0 & 0x00ff0000) | id | IMAPX_OPOSITE_BIT)
#define IMAPX_GPN_OP(id)	((IMAPX_GPN0 & 0x00ff0000) | id | IMAPX_OPOSITE_BIT)
#define IMAPX_GPO_OP(id)	((IMAPX_GPO0 & 0x00ff0000) | id | IMAPX_OPOSITE_BIT)
#define IMAPX_GPP_OP(id)	((IMAPX_GPP0 & 0x00ff0000) | id | IMAPX_OPOSITE_BIT)
#define IMAPX_GPQ_OP(id)	((IMAPX_GPQ0 & 0x00ff0000) | id | IMAPX_OPOSITE_BIT)
#define IMAPX_GPR_OP(id)	((IMAPX_GPR0 & 0x00ff0000) | id | IMAPX_OPOSITE_BIT)
/* GPIO number name operations. */
#define IMAPX_OPOSITE(gpio)	(gpio | IMAPX_OPOSITE_BIT)
#define IMAPX_IS_OPOSITE(gpio)	(gpio & IMAPX_OPOSITE_BIT)
/* GPIO group name bat process. */
#define IMAPX_GPA_ALL		(IMAPX_GPA0 | IMAPX_ALL_IN_GROUP)
#define IMAPX_GPB_ALL		(IMAPX_GPB0 | IMAPX_ALL_IN_GROUP)
#define IMAPX_GPC_ALL		(IMAPX_GPC0 | IMAPX_ALL_IN_GROUP)
#define IMAPX_GPD_ALL		(IMAPX_GPD0 | IMAPX_ALL_IN_GROUP)
#define IMAPX_GPE_ALL		(IMAPX_GPE0 | IMAPX_ALL_IN_GROUP)
#define IMAPX_GPF_ALL		(IMAPX_GPF0 | IMAPX_ALL_IN_GROUP)
#define IMAPX_GPG_ALL		(IMAPX_GPG0 | IMAPX_ALL_IN_GROUP)
#define IMAPX_GPH_ALL		(IMAPX_GPH0 | IMAPX_ALL_IN_GROUP)
#define IMAPX_GPI_ALL		(IMAPX_GPI0 | IMAPX_ALL_IN_GROUP)
#define IMAPX_GPJ_ALL		(IMAPX_GPJ0 | IMAPX_ALL_IN_GROUP)
#define IMAPX_GPK_ALL		(IMAPX_GPK0 | IMAPX_ALL_IN_GROUP)
#define IMAPX_GPL_ALL		(IMAPX_GPL0 | IMAPX_ALL_IN_GROUP)
#define IMAPX_GPM_ALL		(IMAPX_GPM0 | IMAPX_ALL_IN_GROUP)
#define IMAPX_GPN_ALL		(IMAPX_GPN0 | IMAPX_ALL_IN_GROUP)
#define IMAPX_GPO_ALL		(IMAPX_GPO0 | IMAPX_ALL_IN_GROUP)
#define IMAPX_GPP_ALL		(IMAPX_GPP0 | IMAPX_ALL_IN_GROUP)
#define IMAPX_GPQ_ALL		(IMAPX_GPQ0 | IMAPX_ALL_IN_GROUP)
#define IMAPX_GPR_ALL		(IMAPX_GPR0 | IMAPX_ALL_IN_GROUP)
/* Range. */
#define IMAPX_GPA_RANGE(start, end)	\
	(IMAPX_GPA0 | IMAPX_RANGE_GROUP | (start << 12) | (end << 8))
#define IMAPX_GPB_RANGE(start, end)	\
	(IMAPX_GPB0 | IMAPX_RANGE_GROUP | (start << 12) | (end << 8))
#define IMAPX_GPC_RANGE(start, end)	\
	(IMAPX_GPC0 | IMAPX_RANGE_GROUP | (start << 12) | (end << 8))
#define IMAPX_GPD_RANGE(start, end)	\
	(IMAPX_GPD0 | IMAPX_RANGE_GROUP | (start << 12) | (end << 8))
#define IMAPX_GPE_RANGE(start, end)	\
	(IMAPX_GPE0 | IMAPX_RANGE_GROUP | (start << 12) | (end << 8))
#define IMAPX_GPF_RANGE(start, end)	\
	(IMAPX_GPF0 | IMAPX_RANGE_GROUP | (start << 12) | (end << 8))
#define IMAPX_GPG_RANGE(start, end)	\
	(IMAPX_GPG0 | IMAPX_RANGE_GROUP | (start << 12) | (end << 8))
#define IMAPX_GPH_RANGE(start, end)	\
	(IMAPX_GPH0 | IMAPX_RANGE_GROUP | (start << 12) | (end << 8))
#define IMAPX_GPI_RANGE(start, end)	\
	(IMAPX_GPI0 | IMAPX_RANGE_GROUP | (start << 12) | (end << 8))
#define IMAPX_GPJ_RANGE(start, end)	\
	(IMAPX_GPJ0 | IMAPX_RANGE_GROUP | (start << 12) | (end << 8))
#define IMAPX_GPK_RANGE(start, end)	\
	(IMAPX_GPK0 | IMAPX_RANGE_GROUP | (start << 12) | (end << 8))
#define IMAPX_GPL_RANGE(start, end)	\
	(IMAPX_GPL0 | IMAPX_RANGE_GROUP | (start << 12) | (end << 8))
#define IMAPX_GPM_RANGE(start, end)	\
	(IMAPX_GPM0 | IMAPX_RANGE_GROUP | (start << 12) | (end << 8))
#define IMAPX_GPN_RANGE(start, end)	\
	(IMAPX_GPN0 | IMAPX_RANGE_GROUP | (start << 12) | (end << 8))
#define IMAPX_GPO_RANGE(start, end)	\
	(IMAPX_GPO0 | IMAPX_RANGE_GROUP | (start << 12) | (end << 8))
#define IMAPX_GPP_RANGE(start, end)	\
	(IMAPX_GPP0 | IMAPX_RANGE_GROUP | (start << 12) | (end << 8))
#define IMAPX_GPQ_RANGE(start, end)	\
	(IMAPX_GPQ0 | IMAPX_RANGE_GROUP | (start << 12) | (end << 8))
#define IMAPX_GPR_RANGE(start, end)	\
	(IMAPX_GPR0 | IMAPX_RANGE_GROUP | (start << 12) | (end << 8))
#define IMAPX_GET_RANGE_START(gpio) ((gpio & 0x0000f000) >> 12)
#define IMAPX_GET_RANGE_END(gpio) ((gpio & 0x00000f00) >> 8)

/* ############################################################################## */

/**
 * Convert string to GPIO number.
 */
unsigned int __imapx_name_to_gpio(const char *gpio_name);

/**
 * Set gpio mode to be sleep or normal.
 *
 * Return:
 * @ If success, return mode set.
 * @ If error, return IG_BASE_DATA_ERROR.
 */
unsigned int imapx_gpio_setslp(unsigned int gpio, unsigned int sleep);
#define imapx_gpio_chmod imapx_gpio_setslp

/**
 * Check gpio is set to be sleep mode or normal mode.
 *
 * Return:
 * @ 0 - Normal mode.
 * @ none zero - Sleep mode.
 */
unsigned int imapx_gpio_is_sleep(unsigned int gpio);

/**
 * Set mode to assigned gpio.
 *
 * Return:
 * @ If success, return the set mode.
 * @ If error, return IG_BASE_CFG_ERROR.
 */
unsigned int imapx_gpio_setcfg(unsigned int gpio, unsigned int cfg, 
		unsigned int sleep);

/**
 * Get mode from assigned gpio.
 *
 * Return:
 * @ If success, return the gpio correspond mode.
 * @ If error, return IG_BASE_CFG_ERROR.
 */
unsigned int imapx_gpio_getcfg(unsigned int gpio, unsigned int sleep);

/**
 * Write data to assigned gpio, first, the correspond gpio must have
 * been set to be output.
 *
 * Param:
 * @ [I] data - Only 0/1 is valid.
 *
 * Return:
 * @ If success, return written data.
 * @ If error, return IG_BASE_DATA_ERROR.
 */
unsigned int imapx_gpio_setpin(unsigned int gpio, unsigned int data, 
		unsigned int sleep);

/**
 * Read data from assigned gpio, first, the correspond gpio must have
 * been set to be input.
 *
 * Return:
 * @ If success, return the gpio correspond data.
 * @ If error, return IG_BASE_DATA_ERROR.
 */
unsigned int imapx_gpio_getpin(unsigned int gpio, unsigned int sleep);

/**
 * Pull up/down gpio PUD. If current gpio is set to be sleep mode, it 
 * will enable PUD in sleep mode.
 */
void imapx_gpio_pull(unsigned int gpio, unsigned int en, 
		unsigned int sleep);

/**
 * Get pull status. If current gpio is set to be sleep mode, it will
 * returns the gpio PUD status in sleep mode.
 *
 * Return:
 * @ IG_BASE_DATA_ERROR - Some errors occured.
 * @ If success, returns the pull status.
 */
unsigned int imapx_gpio_getpull(unsigned int gpio, unsigned int sleep);

/**
 * Get gpio correspond irq number, the returned irq number is 
 * the system irq number.
 *
 * Return:
 * @ If success, returns correspond irq number.
 * @ If error occurs, returns -1.
 */
int imapx_gpio_to_irq(unsigned int gpio);

/**
 * Set and config external interrupt or group.
 *
 * Param:
 * @ [I] mode - Interrupt generated mode.
 * @ [I] en - Enable irq(unmask) at this function or not.
 *
 * Return:
 * @ If success, returns the mode set.
 * @ If error, returns IG_BASE_DATA_ERROR.
 */
unsigned int imapx_gpio_setirq(unsigned int gpio, unsigned int flt, 
		unsigned int mode, unsigned int en);

/**
 * Check whether gpio correspond irq is pending.
 *
 * Param:
 * @ [I] clear - Clear the pending bit or not.
 *
 * Return:
 * @ If success, returns irq status.
 * @ If error, returns IG_BASE_DATA_ERROR.
 */
unsigned int imapx_gpio_is_pending(unsigned int gpio, unsigned int clear);

/**
 * Clear a gpio correspond irq pending bit.
 */
void imapx_gpio_clear_pend(unsigned int gpio);

/**
 * Unmask a gpio correspond irq.
 */
void imapx_gpio_unmask_irq(unsigned int gpio);

/**
 * Mask a gpio correspond irq.
 */
void imapx_gpio_mask_irq(unsigned int gpio);
