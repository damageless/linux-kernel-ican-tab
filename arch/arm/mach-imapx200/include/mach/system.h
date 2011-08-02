/* arch/arm/mach-imap/include/mach/system.h
 *
 * Copyright (c) 2003 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * IMAPX200 - System function defines and includes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/io.h>
#include <mach/imap_addr.h>
#include <mach/idle.h>
#include <plat/regs-clock.h>
#include <mach/reset.h>
#include <mach/imapx_sysmgr.h>
void (*imap_idle)(void);
void (*imap_reset_hook) (void);

void imap_default_idle(void)
{
	/* idle the system by using the idle mode which will wait for an
	 * interrupt to happen before restarting the system.
	 */

	/* Warning: going into idle state upsets jtag scanning */
	
}

static void arch_idle(void)
{
	if (imap_idle != NULL)
		(imap_idle)();
	else
		imap_default_idle();
}

//#include <mach/system-reset.h>
static void arch_reset(char mode, const char *cmd)
{

	unsigned long tmp;

	__raw_writel(__raw_readl(rMD_ISO)|(0x1)<<4, rMD_ISO);
	__raw_writel(__raw_readl(rNPOW_CFG) & (~(0x1<<4)), rNPOW_CFG);

	tmp = 0x6565;
	if(cmd
	   && (cmd[0] == 'r')
	   && (cmd[1] == 'e')
	   && (cmd[2] == 'c')
	   && (cmd[3] == 'o')
	   && (cmd[4] == 'v')
	   && (cmd[5] == 'e')
	   && (cmd[6] == 'r'))
	{
		if(cmd[7] == '2')
		  __raw_writel(0x00497557, rINFO3);
		else if(cmd[7] == '3')
		  __raw_writel(0x55497557, rINFO3);
		else if(cmd[7] == '4')
		  __raw_writel(0x66497557, rINFO3);
		else if(cmd[7] == '5')
		  __raw_writel(0x77497557, rINFO3);
		else
		  __raw_writel(0xad000001, rINFO3);
		printk(KERN_EMERG "sysreboot: recovery is setted.\n");
	}
	__raw_writel(tmp, rSW_RST);
}

static void arch_shutdown(void)
{

	unsigned long tmp;

	__raw_writel(0xff, rWP_MASK);
	__raw_writel(0x4, rGPOW_CFG);
	
	tmp = 0;
	asm("b 1f\n\t"
	    ".align 5\n\t"
	    "1:\n\t"
	    "mcr p15, 0, %0, c7, c0, 4" :: "r" (tmp));


}
