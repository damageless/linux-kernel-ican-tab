/***************************************************************************** 
 * /linux/arch/arm/mach-imapx200/cpu.c
 * ** 
 * ** Copyright (c) 2009~2014 ShangHai Infotm Ltd all rights reserved. 
 * ** 
 * ** Use of Infotm's code is governed by terms and conditions 
 * ** stated in the accompanying licensing statement. 
 * ** 
 * ** Description: Inialization of the board related fuctions and devices.
 * **
 * ** Author:
 * **     Alex Zhang   <tao.zhang@infotmic.com.cn>
 * **      
 * ** Revision History: 
 * ** ----------------- 
 * ** 1.2  25/11/2009  Alex Zhang   
 * *****************************************************************************/ 

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/sysdev.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/input.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <asm/hardware/iomd.h>
#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <asm/proc-fns.h>
#include <asm/cpu-single.h>
#include <plat/clock.h>
#include <plat/cpu.h>
#include <plat/devs.h>
#include <plat/imapx200.h>
#include <mach/idle.h>
#include <plat/imapx.h>
#include <plat/regs-serial.h>
#include <plat/pm.h>
#include <linux/amba/bus.h>



#define DEF_MCR	IMAPX200_MCR_SIRE_IRDA_DISABLE | IMAPX200_MCR_AFCE_AFC_DISABLE
#define DEF_LCR IMAPX200_LCR_DLS_8BIT |IMAPX200_LCR_STOP_ONE_STOP_BIT |IMAPX200_LCR_PEN_PARITY_DISABLE
#define DEF_FCR  IMAPX200_FCR_FIFOE_FIFO_ENABLE | IMAPX200_FCR_TET_TX_THRESHOLD_LEVEL_EMPTY | IMAPX200_FCR_RT_RX_TRIGGER_LEVEL_ONE_CHAR 

extern void imap_mem_reserve(void);
extern struct sys_timer imapx200_init_timer;

static struct platform_device *imapx200_devices[] __initdata = {
	&imapx200_device_nand,	
	&imapx200_device_lcd,
	&imapx200_device_pwm,
	&imapx200_device_bl,
	&imapx200_button_device,
	&imapx200_device_sdi0,
	&imapx200_device_sdi1,
	&imapx200_device_sdi2,
	&imapx200_device_cf,
	&imapx200_device_usbhost11,
	&imapx200_device_usbhost20,
	&imapx200_device_usbotg,
	&imapx200_device_udc,
	&imapx200_device_camera,
#ifdef CONFIG_RGB2VGA_OUTPUT_SUPPORT
	&imapx200_device_rgb2vga,
#endif
#ifdef CONFIG_HDMI_EP932
	&imapx200_device_hdmi_ep932,
#endif
#ifdef CONFIG_HDMI_SI9022
	&imapx200_device_hdmi_si9022,
#endif
	&imapx200_device_mac,
	&imapx200_device_rda5868,
	&imapx200_device_venc,
	&imapx200_device_vdec,
	&imapx200_device_memalloc,
	&imapx200_device_dsp,
	&imapx200_device_gps,
	&imapx200_device_iic0,
	&imapx200_device_iic1,
	&imapx200_device_rtc,
	&imapx200_device_iis,
	&imapx200_device_ac97,
	&imapx200_device_ssim0,
	&imapx200_device_ssim1,
	&imapx200_device_ssim2,
	&imapx200_device_ssis,
	&imapx200_device_spi,
	&imapx200_device_keybd,
	&imapx200_device_pic0,
	&imapx200_device_pic1,
	&imapx200_device_graphic,
	&imapx200_device_orientation,
	&imapx200_device_backlights,
	&imapx200_devcie_watchdog,
	&imapx200_devcie_mtouch,
#ifdef CONFIG_ANDROID_PMEM
	&android_pmem_device,
	&android_pmem_adsp_device,
#endif
#ifdef	CONFIG_ANDROID_TIMED_GPIO
	&android_vibrator_device,
#endif
#ifdef CONFIG_BOSCH_BMA150
	&imapx200_device_BMA150,
#endif
	&android_wifi_switch_device,
#ifdef CONFIG_SWITCH_SENSOR
	&android_sensor_switch_device,
#endif
#ifdef	CONFIG_USB_ANDROID
	&android_usb_device,
#endif	
	&android_device_batt,
};

static struct map_desc imapx200_iodesc[] __initdata = {
};


static struct imapx200_uart_clksrc imapx200_serial_clocks[] = {

	[0] = {
		.name		= "pclk",
		.divisor	= 1,
		.min_baud	= 0,
		.max_baud	= 0,
	},

#if 0 /* HS UART Source is changed from epll to mpll */
	[1] = {
		.name		= "ext_uart",
		.divisor	= 1,
		.min_baud	= 0,
		.max_baud	= 0,
	},

#if defined (CONFIG_SERIAL_S3C64XX_HS_UART)

	[2] = {
		.name		= "epll_clk_uart_192m",
		.divisor	= 1,
		.min_baud	= 0,
		.max_baud	= 4000000,
	}
#endif

#endif

};


static struct imapx200_uartcfg imapx200_uartcfgs[] = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.fcr	     = DEF_FCR,
		.lcr	     = DEF_LCR,
		.mcr	     = DEF_MCR,
		.clocks	     = imapx200_serial_clocks,
		.clocks_size = ARRAY_SIZE(imapx200_serial_clocks),
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.fcr	     = DEF_FCR,
		.lcr	     = DEF_LCR,
		.mcr	     = DEF_MCR,
		.clocks	     = imapx200_serial_clocks,
		.clocks_size = ARRAY_SIZE(imapx200_serial_clocks),
	},
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.fcr	     = DEF_FCR,
		.lcr	     = DEF_LCR,
		.mcr	     = DEF_MCR,
		.clocks	     = imapx200_serial_clocks,
		.clocks_size = ARRAY_SIZE(imapx200_serial_clocks),
	},
#if 1
	[3] = {
		.hwport	     = 3,
		.flags	     = 0,
		.fcr	     = DEF_FCR,
		.lcr	     = DEF_LCR,
		.mcr	     = DEF_MCR,
		.clocks	     = imapx200_serial_clocks,
		.clocks_size = ARRAY_SIZE(imapx200_serial_clocks),
	}
#endif
};

static void imapx200_idle(void)
{
	unsigned long tmp;
	/* ensure our idle mode is to go to idle */

	/* Set WFI instruction to SLEEP mode */
#if 1
	tmp = __raw_readl(rGPOW_CFG);
	tmp &= ~(0x7);
	tmp |= 0x1;
//	__raw_writel(tmp, rGPOW_CFG);
	cpu_do_idle();
#endif
}

struct sysdev_class imapx200_sysclass = { 
	.name           = "imapx200-core",
};

static struct sys_device imapx200_sysdev = { 
	.cls            = &imapx200_sysclass,
};

static int __init imapx200_core_init(void)
{
	return sysdev_class_register(&imapx200_sysclass);
}
core_initcall(imapx200_core_init);

void __init imapx200_init_irq(void)
{
	imap_init_irq();
}

void __init imdkx200_map_io(struct map_desc *mach_desc, int size)
{
	/* register our io-tables */
	iotable_init(mach_desc, size);
	/* rename any peripherals used differing from the s3c2412 */
	imap_idle = imapx200_idle;
}

void __init imapx200_init_uarts(struct imapx200_uartcfg *cfg, int no) 
{
	unsigned long tmp;    
	imap_init_uartdevs("imapx200-uart", imapx200_uart_resources, cfg, no);

	//      imap_device_lcd.name = "imap-lcd";
	//      imap_device_nand.name = "imap-nand";
}

int __init imapx200_init(void)
{
	int ret;

	printk("imapx200: Initialising architecture\n");

	ret = sysdev_register(&imapx200_sysdev);

	if(ret != 0)
		printk(KERN_ERR "failed to register sysdev for iampx200\n");
	return ret;
}

static struct imap_board imapx200_board __initdata = { 
	.devices        = imapx200_devices,
	.devices_count  = ARRAY_SIZE(imapx200_devices)
};

static void __init imapx200_map_io(void)
{
	imap_init_io(imapx200_iodesc, ARRAY_SIZE(imapx200_iodesc));
	imap_init_clocks(0);
	imap_init_uarts(imapx200_uartcfgs, ARRAY_SIZE(imapx200_uartcfgs));
	imap_mem_reserve();
	imap_pm_init();
}


static void __init imapx200_fixup(struct machine_desc *desc, struct tag *tags,
		char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 1;
	mi->bank[0].start = 0x40000000;
	mi->bank[0].size = 256*1024*1024;
	mi->bank[0].node = 0;
}

static struct platform_device fsg_platform_device = 
{
	.name = "usb_mass_storage",
	.id = -1,
};

static void __init imapx200_machine_init(void)
{
	platform_add_devices(imapx200_devices, ARRAY_SIZE(imapx200_devices));
	amba_device_register(&imap_ps2_device, &(imap_ps2_device.res));
	(void)platform_device_register(&fsg_platform_device);
}

#if defined(CONFIG_MACH_IMAPX200)
MACHINE_START(IMAPX200, "IMAPX200")
#endif
	.phys_io	= UART0_BASE_ADDR,
	.io_pg_offst	= (((u32)UART0_BASE_ADDR) >> 18) & 0xfffc,
	.boot_params	= IMAPX200_SDRAM_PA + 0x100,

	.init_irq	= imdkx200_init_irq,
	.map_io		= imapx200_map_io,
	.fixup		= imapx200_fixup,
	.timer		= &imapx200_init_timer,
	.init_machine	= imapx200_machine_init,
MACHINE_END	
