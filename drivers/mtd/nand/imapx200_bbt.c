/***************************************************************************** 
** XXX driver/mtd/imapx200_bbt.c XXX
** 
** Copyright (c) 2009~2014 ShangHai Infotm Ltd all rights reserved. 
** 
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
** 
** Description: iMAP NAND Flash platform driver.
**				TODO: Science spare ECC is used, read_oob, wirte_oob
**					should be applied to ensure the validity of OOB.
**
** Author:
**     Jay		<jay.hu@infotmic.com.cn>
**     Warits	<warits.wang@infotmic.com.cn>
**      
** Revision History: 
** ----------------- 
** 1.1  XXX 11/25/2010 XXX	Initialized by Jay
** 1.1  XXX 11/25/2010 XXX	Add nor functions by Warits
*****************************************************************************/

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>
#include <linux/highmem.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>

#define __ibbt_msg(args...) printk(KERN_ERR "iMAPx200 BBT: " args)
#define __ibbt_dbg(args...) printk(KERN_ERR "iMAPx200 BBT: " args)
//#define __ibbt_dbg(args...)
//#define __nand_msg(args...)

#define IMAP_BBT_MAGIC			(0xbadb10cc)
#define IMAP_BBT_SIZE			(0x2000)
#define IMAP_BBT_OFFS			(0x38000)
#define IMAP_BBT_MOFFS			(0x39000)
#define IMAP_NOR_SIZE			(0x80000)
#define IMAP_NOR_BW_SHIFT		1		

/**********
 * NOR BBT:  8KB, 0xffff: good block, 0x00ee: initial bad block, 0x0011: fs marked bad block
 * RAM BBT:  4KB,   0x00: good block,   0xee: initial bad block,   0x11: fs marked bad block
 **********
 */

static uint8_t *imap_bbt = NULL;
static void  __iomem *imap_nor_base; 
extern int nand_default_block_markbad(struct mtd_info *mtd, loff_t ofs);
enum IMAP_BBT_TYPE {
	IMAP_BBT_TYPE_INIT,
	IMAP_BBT_TYPE_MARK,
};

static int imap_bbt_markbad(uint32_t blk, enum IMAP_BBT_TYPE type)
{
	switch(type)
	{
		case IMAP_BBT_TYPE_INIT:
			*(uint16_t *)(imap_bbt + blk) = 0xee;
			break;
		case IMAP_BBT_TYPE_MARK:
			*(uint16_t *)(imap_bbt + blk) = 0x11;
			break;
	}

	return 0;
}


/* NOR Flash interface */
static void inline nor_cmd(uint32_t addr, uint16_t val)
{
	void __iomem *b_addr = (imap_nor_base + (addr << IMAP_NOR_BW_SHIFT));
	writew(val, b_addr);
}

static uint16_t nor_get(uint32_t addr)
{
	uint32_t b_addr = (uint32_t)(imap_nor_base + (addr << IMAP_NOR_BW_SHIFT));
	return readw(b_addr);
}

static int nor_bit_toggling(uint32_t addr)
{
	uint16_t q6, t = 4;

	/* check if Q6 is toggling */
	q6 = nor_get(addr) & 0x40;
	while(t--)
	  if((nor_get(addr) & 0x40) != q6)
		/* bit is toggling */
		return 1;

	return 0;
}

static int nor_word_prog(uint32_t addr, uint16_t word)
{
	nor_cmd(0x555, 0xaa);
	nor_cmd(0x2aa, 0x55);
	nor_cmd(0x555, 0xa0);
	nor_cmd(addr, word);

	while(nor_bit_toggling(addr));

	return 0;
}

static int nor_id(int entry)
{
	nor_cmd(0x555, 0xaa);
	nor_cmd(0x2aa, 0x55);

	if(entry)
	  nor_cmd(0x555, 0x90);
	else
	  nor_cmd(0x555, 0xf0);

	udelay(2);
	return 0;
}

static void nor_reset(void)
{
	int a;
	__ibbt_msg("Reseting NOR flash...\n");

	imapx_gpio_setcfg(IMAPX_GPH_ALL, IG_CTRL0, IG_NORMAL);
	imapx_gpio_setcfg(IMAPX_GPI_ALL, IG_CTRL0, IG_NORMAL);
	imapx_gpio_setcfg(IMAPX_GPJ_ALL, IG_CTRL0, IG_NORMAL);

	a = *(int *)imap_nor_base;
	*(int *)imap_nor_base = 0;

	nor_id(1);
	__ibbt_msg("Maf: %x, Did: %x\n",
	   (uint32_t)nor_get(0), (uint32_t)nor_get(1));
	nor_id(0);
	return ;
}

static int imap_nor_markbad(uint32_t block)
{
	if(block > IMAP_BBT_SIZE / sizeof(short))
	  __ibbt_msg("Block count 0x%x exceed BBT limit.\n", block);

	nor_word_prog(IMAP_BBT_OFFS + block, 0x11);
	__ibbt_msg("Block count 0x%x marked as bad.\n", block);

	return 0;
}

static uint32_t imap_nor_getmagic(void)
{
	uint32_t magic;

	magic = nor_get(IMAP_BBT_MOFFS) | (nor_get(IMAP_BBT_MOFFS + 1) << 16);
	return magic;
}

static int imap_nor_gettable(void)
{
	int i;
	uint16_t stat;
	if(imap_nor_getmagic() != IMAP_BBT_MAGIC)
	{
		__ibbt_msg("No magic found, get BBT failed.\n");
		return -1;
	}

	for(i = 0; i < (IMAP_BBT_SIZE >> 1); i++)
	{
		stat = nor_get(IMAP_BBT_OFFS + i);
		if(stat != 0xffff)
		{
			stat &= 0xff;
			*(imap_bbt + i) = stat;
			__ibbt_msg("Bad block 0x%x(%s) found.\n", i,
			   ((stat == 0xee) ? "IB" :
				((stat == 0x11)?"MB" : "UNKNOWN")));
		}
	}
	return 0;
}

static int imap_nor_bbtsync(struct mtd_info *mtd)
{
	int i, ret;
	uint16_t stat;
	struct nand_chip *chip = mtd->priv;

	for(i = 0; i < IMAP_BBT_SIZE / sizeof(uint16_t); i++)
	{
		ret = nand_isbad_bbt(mtd, i << chip->bbt_erase_shift, 0);

		if(ret)
		{
			imap_bbt_markbad(i, IMAP_BBT_TYPE_INIT);
			/* Mark as initial bad */
			nor_word_prog(IMAP_BBT_OFFS + i, 0xee);
			stat = nor_get(IMAP_BBT_OFFS + i);

			if(stat != 0xee)
			{
				__ibbt_msg("Mark initial bad failed!\n");
				return -1;
			}
		} else {
			stat = nor_get(IMAP_BBT_OFFS + i);
			if(stat != 0xffff)
			{
				__ibbt_msg("NOR is not in 0xffff stat(%x), but BBT need reprsent good block!\n",
				   stat);
				__ibbt_msg("BBT sync failed at block 0x%x\n", i);
				return -1;
			}
		}
	}

	return 0;
}


static int imap_nor_setmagic(void)
{
	uint32_t magic;

	magic = imap_nor_getmagic();

	if(magic == IMAP_BBT_MAGIC)
		__ibbt_msg("Magic already setted.\n");
	else if(magic == 0xffffffff)
	{
		nor_word_prog(IMAP_BBT_MOFFS, IMAP_BBT_MAGIC & 0xffff);
		nor_word_prog(IMAP_BBT_MOFFS + 1,
		   (IMAP_BBT_MAGIC >> 16) & 0xffff);
		__ibbt_msg("Magic setted.\n");

		return 0;
	} else
	  __ibbt_msg("NOR is in wrong state to set magic.\n");

	return -1;
}

static int imap_nor_init(void)
{
	struct clk *clk;

	imap_nor_base = ioremap_nocache(0x10000000, IMAP_NOR_SIZE);

	if(imap_nor_base == NULL)
	{
		__ibbt_msg("Can not map NOR address.\n");
		return -EIO;
	}

	clk = clk_get(NULL, "norflash");
	if(IS_ERR(clk))
	{
		__ibbt_msg("Failed to get clock.\n");
		return -ENOENT;
	}

	clk_enable(clk);

	nor_reset();
	return 0;
}

static int imap_nor_exit(void)
{
	if(imap_nor_base)
	  iounmap(imap_nor_base);

	__ibbt_msg("NOR memory area released.\n");
	return 0;
}

/*----------- This is a break line -------------*/



/* BBT interface */
/* This function is only be invoked when chip->bbt is disabled */
int imap_block_bad(struct mtd_info *mtd, loff_t offs, int allowbbt)
{

	struct nand_chip *chip = mtd->priv;
	uint8_t pt = 0;
	uint32_t block;

	block = (int)(offs >> chip->bbt_erase_shift);
	pt = *(imap_bbt + block);

	switch(pt)
	{
		case 0:
			break;
		case 0xee:
			__ibbt_msg("Bad block detected(IB): 0x%x\n", block);
			return 1;
		case 0x11:
			__ibbt_msg("Bad block detected(MB): 0x%x\n", block);
			return 1;
		default:
			__ibbt_msg("Unrecogenized mark! (Will not be reported as a bad block)\n");
			break;
	}

	return 0;
}

int imap_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *chip = mtd->priv;

	int block, ret;

	/* Also mark physical NAND bad */
	ret = nand_default_block_markbad(mtd, ofs);

	if(ret)
	  __ibbt_msg("Default mark to NAND failed.\n");

	if(imap_bbt)
	{
		block = (int)(ofs >> chip->bbt_erase_shift);

		/* Mark this block as bad */
		imap_nor_markbad(block);
		imap_bbt_markbad(block, IMAP_BBT_TYPE_MARK);

		/* We successed on mark bad on NOR */
		return 0;
	}

	return ret;
}

int imap_scan_bbt(struct mtd_info *mtd)
{
	int count, ret;
	uint32_t magic;
	struct nand_chip *chip = mtd->priv;

	printk(KERN_INFO "iMAPx200 NAND NOR BBT (c) 2009,2014 InfoTM\n");

	if(imap_bbt)
	{
		__ibbt_msg("iMAP BBT started with none NULL value!\n");

		/* FIXME: This is for debugging, remove this if a release version */
		BUG();
	}

	imap_bbt = kzalloc(IMAP_BBT_SIZE >> 1, GFP_KERNEL);
	if(imap_bbt == NULL){
		__ibbt_msg("Allocating %x for imap BBT failed!\n", IMAP_BBT_SIZE >> 1);
		goto __nor_bbt_failed__;
	}

	count = mtd->size >> chip->bbt_erase_shift;

	ret = imap_nor_init();
	if(ret)
	  goto __nor_bbt_failed__;

	magic = imap_nor_getmagic();
	if(magic == IMAP_BBT_MAGIC)
	{
		__ibbt_msg("NOR based BBT found.\n");
		ret = imap_nor_gettable();
		if(ret)
		  goto __nor_bbt_failed__;
	}
	else
	{
		__ibbt_msg("Creating NOR based BBT ...\n");
		
		ret = nand_default_bbt(mtd);
		if(ret)
		  goto __nor_bbt_failed__;

		ret = imap_nor_bbtsync(mtd);
		if(ret)
		  goto __nor_bbt_failed__;

		/* This means the NOR bbt is available and the default BBT
		 * will not be used any more.
		 */
		ret = imap_nor_setmagic();
		if(ret)
		  goto __nor_bbt_failed__;
	}

	/* Nor BBT initialize successful, Disable default BBT */
	chip->bbt = NULL;
	return 0;

__nor_bbt_failed__:
	if(imap_bbt)
	  kfree(imap_bbt);
	imap_nor_exit();

	/* FIXME: We should not return an ERROR */
	return 0;
}

EXPORT_SYMBOL_GPL(imap_block_markbad);
EXPORT_SYMBOL_GPL(imap_scan_bbt);
EXPORT_SYMBOL_GPL(imap_block_bad);
