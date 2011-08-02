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
//#define __nand_dbg(args...) printk(KERN_ERR "iMAPx200 BBT: " args)
#define __ibbt_dbg(args...)
//#define __nand_msg(args...)

#define IMAP_BBT_MAGIC			(0xbadb10cc)
#define IMAP_BBT_SIZE			(0x2000)
#define IMAP_BBT_OFFS			(0x30000)
#define IMAP_BBT_MOFFS			(0x31000)
#define IMAP_NOR_SIZE			(0x80000)
#define IMAP_NOR_BW_SHIFT		1		

static uint8_t *imap_bbt = NULL;

/* NOR Flash interface */
static void inline nor_cmd(uint32_t addr, uint16_t val)
{
	uint32_t b_addr = (imap_nor_base + (addr << CONFIG_NOR_BW_SHIFT));
	writew(val, b_addr);
}

static uint16_t nor_get(uint32_t addr)
{
	uint32_t b_addr = (imap_nor_base + (addr << CONFIG_NOR_BW_SHIFT));
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
	__ibbt_msg("Reseting NOR flash...\n");

	writel(0xaa, rGPHCON);
	writel(0xaaaaaaa, rGPICON);
	writel(0x2aaaa, rGPJCON);

	nor_id(1);
	__ibbt_msg("Maf: %x, Did: %x\n",
	   (uint32_t)nor_get(0), (uint32_t)nor_get(1));
	nor_id(0);
	return ;
}

static int imap_nor_markbad(uint32_t block)
{
	if(block > IMAP_BBT_SIZE / sizeof(short))
	  __ibbt_msg("Block count %d exceed BBT limit.\n", block);

	nor_word_prog(IMAP_BBT_OFFS + block, 0x11);
	__ibbt_msg("Block count %d marked as bad.\n", block);

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
	if(imap_nor_getmagic() != IMAP_BBT_MAGIC)
	{
		__ibbt_msg("No magic found, get bbt failed.\n");
		return -1;
	}

	memcpy(imap_bbt, imap_nor_base, IMAP_BBT_SIZE);
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
	imap_nor_base = ioremap(0x10000000, IMAP_NOR_SIZE);

	if(imap_nor_base == NULL)
	{
		__ibbt_msg("Can not remap NOR address.\n");
		return -EIO;
	}

	nor_reset();
	return 0;
}

/*----------- This is a break line -------------*/



/* BBT interface */
int imap_block_bad(struct mtd_info *mtd, loff_t offs, int allowbbt)
{

	struct nand_chip *chip = mtd->priv;
	uint16_t pt = 0;

	block = (int)(offs >> chip->bbt_erase_shift);
	pt = *(uint16_t *)(imap_bbt + (block << 1));

	switch(pt)
	{
		case 0xffff:
			break;
		case 0x00:
			__ibbt_msg("Bad block detected(IB): %08x\n", offs);
			return 1;
		case 0x11:
			__ibbt_msg("Bad block detected(MB): %08x\n", offs);
			return 1;
		default:
			__ibbt_msg("Unrecogenized mark!(Will not be reported as a bad block)\n");
			break;
	}

	return 0;
}

int imap_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *chip = mtd->priv;

	int block, ret;

	block = (int)(ofs >> chip->bbt_erase_shift);

	/* Mark this block as bad */
	*(uint16_t *)(imap_bbt + (block << 1)) = 0x11;

	imap_nor_markbad(block);

	/* Also mark physical NAND bad */
	ret = chip->block_markbad(mtd, ofs);

	return ret;
}

int imap_scan_bbt(struct mtd_info *mtd)
{
	int i = 0, len = 0, startblock, count;
	int magic, offset, blkcount = 0;
	unsigned char *buf;
	struct mtd_oob_ops ops;
	struct nand_bbt_descr *bd;
	struct nand_chip *chip = mtd->priv;
	loff_t from;

	if(bd == NULL){
		__ibbt_msg("Unable to get block patten! iMAP BBT created failed.\n");
		return -1;
	}

	imap_bbt = kzalloc(IMAP_BBT_SIZE, GFP_KERNEL);
	if(imap_bbt == NULL){
		__ibbt_msg("Allocating %x for imap BBT failed!\n", IMAP_BBT_SIZE);
		return -1;
	}

	bd = chip->badblock_pattern;
	count = mtd->size / mtd->erasesize;

	/* Disable default BBT */
	chip->bbt = NULL;

	magic = imap_nor_getmagic();
	if(magic == IMAP_BBT_MAGIC)
	{
		__ibbt_msg("NOR based BBT found.\n");
		imap_nor_gettable();
	}
	else
	{
		__ibbt_msg("Creating NOR based BBT ...\n");
		
		buf = kzalloc(mtd->oobsize, GFP_KERNEL);

		from  = 0;
		for(i = 0; i < count; i++)
		{
			ret = scan_block_fast(mtd, bd, from, buf, 2);
			if(ret)
			  imap_nor_markbad(from >> chip->bbt_erase_shift);
		
			i += 2;
			from += (1 << chip->bbt_erase_shift);
		}

		imap_nor_gettable();
		imap_nor_setmagic();
	}

	return 0;
}

EXPORT_SYMBOL_GPL(imap_block_markbad);
EXPORT_SYMBOL_GPL(imap_scan_bbt);
EXPORT_SYMBOL_GPL(imap_block_bad);
