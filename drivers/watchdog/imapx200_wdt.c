/***************************************************************************** 
 * ** drivers/watchdog/imapx200-wdt.c
 * ** 
 * ** Copyright (c) 2009~2014 ShangHai Infotm Ltd all rights reserved. 
 * ** 
 * ** This program is free software; you can redistribute it and/or modify
 * ** it under the terms of the GNU General Public License as published by
 * ** the Free Software Foundation; either version 2 of the License, or
 * ** (at your option) any later version.
 * ** 
 * ** Description: Watchdog driver for iMAPx200
 * **
 * ** Author:
 * **     Jay   <jay.hu@infotm.com>
 * **      
 * ** Revision History: 
 * ** ----------------- 
 * ** 1.1  XXX 09/18/2009 XXX      Initialized
 * ** 1.2  XXX 12/10/2010 XXX      Add reboot mode
 * *****************************************************************************/


#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/io.h>



/* the watchdog can either generate a reset pulse, or an
 *  * interrupt.
 *   */

#define IMAPX200_WTCON_INTEN   (1<<1)
#define IMAPX200_WTCON_ENABLE  (1<<0)

#include <plat/regs-watchdog.h>

#define PFX "imapx200-wdt: "

#define CONFIG_IMAPX200_WATCHDOG_ATBOOT		(0)
#define CONFIG_IMAPX200_WATCHDOG_DEFAULT_TIME	(15)

#define CONFIG_IMAPX200_FRIST_RESET	(0x66626f74)
#define CONFIG_IMAPX200_SECOND_RESET	(0x73626f74)
#define CONFIG_IMAPX200_REQ_REASE	(0x72656173)


static int nowayout	= WATCHDOG_NOWAYOUT;
static int tmr_margin	= CONFIG_IMAPX200_WATCHDOG_DEFAULT_TIME;
static int tmr_atboot	= CONFIG_IMAPX200_WATCHDOG_ATBOOT;
static int soft_noboot;
static int debug;

module_param(tmr_margin,  int, 0);
module_param(tmr_atboot,  int, 0);
module_param(nowayout,    int, 0);
module_param(soft_noboot, int, 0);
module_param(debug,	  int, 0);

MODULE_PARM_DESC(tmr_margin, "Watchdog tmr_margin in seconds. default="
		__MODULE_STRING(CONFIG_IMAPX200_WATCHDOG_DEFAULT_TIME) ")");
MODULE_PARM_DESC(tmr_atboot,
		"Watchdog is started at boot time if set to 1, default="
			__MODULE_STRING(CONFIG_IMAPX200_WATCHDOG_ATBOOT));
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
			__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");
MODULE_PARM_DESC(soft_noboot, "Watchdog action, set to 1 to ignore reboots, "
			"0 to reboot (default depends on ONLY_TESTING)");
MODULE_PARM_DESC(debug, "Watchdog debug, set to >1 for debug, (default 0)");

static unsigned long open_lock;
static struct device    *wdt_dev;	/* platform device attached to */
static struct resource	*wdt_mem;
static struct resource	*wdt_irq;
static struct clk	*wdt_clock;
static void __iomem	*wdt_base;
static unsigned int	 wdt_count;
static char		 expect_close;
static DEFINE_SPINLOCK(wdt_lock);


struct hrtimer wdt_feed_timer;
static struct timeval wdt_time_start;
static struct timeval wdt_time_end;
static int wdt_time;

static int kill_reboot;

/* watchdog control routines */

#define DBG(msg...) do { \
	if (debug) \
		printk(KERN_INFO msg); \
	} while (0)

/* functions */


static void imapx200wdt_stop(void)
{

	writel(0x0, wdt_base + WDT_CR);

}

static void imapx200wdt_start(void)
{
	unsigned long wtcon;


	writel(0x1, wdt_base + WDT_CR);

}

static int imapx200wdt_set_heartbeat(int timeout)
{

	printk("In function %s line %d\n",__func__,__LINE__);

	if(timeout > 15 || timeout < 0)
	{
		return -1;
	}

	writel(timeout, wdt_base + WDT_TORR);
	writel(0x76, wdt_base + WDT_CRR);
	writel(0x1, wdt_base + WDT_CR);

	return 0;
}

static enum hrtimer_restart wdt_hrtimer(struct hrtimer *handle)
{

	uint32_t boot = 0;

	wdt_time++;

	if(kill_reboot == 0)
	{	
		if(wdt_time < CONFIG_IMAPX200_WATCHDOG_RESET_SEC)
		{	

			//imapx200wdt_set_heartbeat(15);

			hrtimer_start(&wdt_feed_timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	//		printk("wdt_hrtimer, wdt_time = %d\n",wdt_time);

		}
		else
		{
			printk("wdt_hrtimer time out , wdt_time = %d\n",wdt_time);
			boot = readl(wdt_base + INFO1);
	
			if(boot == CONFIG_IMAPX200_SECOND_RESET)
			{
				writel(CONFIG_IMAPX200_REQ_REASE, wdt_base + INFO1);	
			}
			else
			{
				writel(CONFIG_IMAPX200_SECOND_RESET, wdt_base + INFO1);
			}

			writel(0x6565, wdt_base + SW_RST);	
		}

	}
	else
	{
		writel(0, wdt_base + INFO1);
	}
	
	return 0;

}

/*
 *	/dev/watchdog handling
 */

static int imapx200wdt_open(struct inode *inode, struct file *file)
{

	printk("In function %s line %d\n",__func__,__LINE__);
	return 0;
}

static int imapx200wdt_release(struct inode *inode, struct file *file)
{
	/*
	 *	Shut off the timer.
	 * 	Lock it in if it's a module and we set nowayout
	 */
	printk("In function %s line %d\n",__func__,__LINE__);
	return 0;
}

static ssize_t imapx200wdt_write(struct file *file, const char __user *data,
				size_t len, loff_t *ppos)
{
	/*
	 *	Refresh the timer.
	 */
	int i = 0;
	uint32_t ret = 0;
	char rbootstr[12] = {"kill reboot"};

	printk("In function %s line %d\n",__func__,__LINE__);


	if(len <= 12)
	{
		for(i=0; i<len - 1; i++)
		{
			if(rbootstr[i] != data[i]){
				ret = -1;
				break;
			}
		}	
		
		if(ret == 0)
        	{
			kill_reboot = 1;
                	printk("watchdog kill reboot\n");
        	}
	}

	if (len) {
		if (!nowayout) {
			size_t i;

			/* In case it was set long ago */
			expect_close = 0;

			for (i = 0; i != len; i++) {
				char c;

				if (get_user(c, data + i))
					return -EFAULT;
				if (c == 'V')
					expect_close = 42;
			}
		}
	}
	return len;
}

#define OPTIONS (WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE)

static const struct watchdog_info imapx200_wdt_ident = {
	.options          =     OPTIONS,
	.firmware_version =	0,
	.identity         =	"IMAPX200 Watchdog",
};


static long imapx200wdt_ioctl(struct file *file,	unsigned int cmd,
							unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_margin;

	printk("In function %s line %d\n",__func__,__LINE__);

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return  0;
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);
	case WDIOC_KEEPALIVE:
		return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_margin, p))
			return -EFAULT;
		if (imapx200wdt_set_heartbeat(new_margin))
			return -EINVAL;
		return put_user(tmr_margin, p);
	case WDIOC_GETTIMEOUT:
		return put_user(tmr_margin, p);
	default:
		return -ENOTTY;
	}
}

/* kernel interface */

static const struct file_operations imapx200wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= imapx200wdt_write,
	.unlocked_ioctl	= imapx200wdt_ioctl,
	.open		= imapx200wdt_open,
	.release	= imapx200wdt_release,
};

static struct miscdevice imapx200wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &imapx200wdt_fops,
};

/* interrupt handler code */

static irqreturn_t imapx200wdt_irq(int irqno, void *param)
{
	dev_info(wdt_dev, "watchdog timer expired (irq)\n");

	return IRQ_HANDLED;
}
/* device interface */

static int __devinit imapx200wdt_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct device *dev;
	unsigned int wtcon;
	int started = 0;
	int ret;
	int size;

	DBG("%s: probe=%p\n", __func__, pdev);

	dev = &pdev->dev;
	wdt_dev = &pdev->dev;

	printk("In function %s line %d\n",__func__,__LINE__);

	/* get the memory region for the watchdog timer */

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "no memory resource specified\n");
		return -ENOENT;
	}

	size = (res->end - res->start) + 1;
	wdt_mem = request_mem_region(res->start, size, pdev->name);
	if (wdt_mem == NULL) {
		dev_err(dev, "failed to get memory region\n");
		ret = -ENOENT;
		goto err_req;
	}

	wdt_base = ioremap(res->start, size);
	if (wdt_base == NULL) {
		dev_err(dev, "failed to ioremap() region\n");
		ret = -EINVAL;
		goto err_req;
	}

	DBG("probe: mapped wdt_base=%p\n", wdt_base);

	wdt_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (wdt_irq == NULL) {
		dev_err(dev, "no irq resource specified\n");
		ret = -ENOENT;
		goto err_map;
	}

	ret = request_irq(wdt_irq->start, imapx200wdt_irq, 0, pdev->name, pdev);
	if (ret != 0) {
		dev_err(dev, "failed to install irq (%d)\n", ret);
		goto err_map;
	}

         ret = misc_register(&imapx200wdt_miscdev);
         if (ret) {
                 dev_err(dev, "cannot register miscdev on minor=%d (%d)\n",
                         WATCHDOG_MINOR, ret);
                 goto err_map;
         }
 
	kill_reboot = 0;
	wdt_time = 0;

	printk("In function %s line %d\n",__func__,__LINE__);

	hrtimer_init(&wdt_feed_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	wdt_feed_timer.function = wdt_hrtimer;

	hrtimer_start(&wdt_feed_timer, ktime_set(0, 50), HRTIMER_MODE_REL);


 err_irq:
	free_irq(wdt_irq->start, pdev);
 err_req:
 err_map:
	return ret;
}

static int __devexit imapx200wdt_remove(struct platform_device *dev)
{
	return 0;
}

static void imapx200wdt_shutdown(struct platform_device *dev)
{
	imapx200wdt_stop();
}

#ifdef CONFIG_PM

static unsigned long wtcon_save;
static unsigned long wtdat_save;

static int imapx200wdt_suspend(struct platform_device *dev, pm_message_t state)
{
	/* Save watchdog state, and turn it off. */

	/* Note that WTCNT doesn't need to be saved. */

	return 0;
}

static int imapx200wdt_resume(struct platform_device *dev)
{
	/* Restore watchdog state. */



	return 0;
}

#else
#define s3c2410wdt_suspend NULL
#define s3c2410wdt_resume  NULL
#endif /* CONFIG_PM */


static struct platform_driver imapx200wdt_driver = {
	.probe		= imapx200wdt_probe,
	.remove		= __devexit_p(imapx200wdt_remove),
	.shutdown	= imapx200wdt_shutdown,
	.suspend	= imapx200wdt_suspend,
	.resume		= imapx200wdt_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "imapx200-wdt",
	},
};


static char banner[] __initdata =
	KERN_INFO "imapx200 Watchdog Timer, (c) 2004 Simtec Electronics\n";

static int __init watchdog_init(void)
{
	printk(banner);
	return platform_driver_register(&imapx200wdt_driver);
}

static void __exit watchdog_exit(void)
{
	platform_driver_unregister(&imapx200wdt_driver);
}

module_init(watchdog_init);
module_exit(watchdog_exit);

MODULE_AUTHOR("Ben Dooks <ben@simtec.co.uk>, "
	      "Dimitry Andric <dimitry.andric@tomtom.com>");
MODULE_DESCRIPTION("IMAPX200 Watchdog Device Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS("platform:imapx200-wdt");
