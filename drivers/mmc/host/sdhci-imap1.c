/* linux/drivers/mmc/host/sdhci-imap.c
 *
 * Copyright 2008 Openmoko Inc.
 * Copyright 2008 Simtec Electronics
 *      Ben Dooks <ben@simtec.co.uk>
 *      http://armlinux.simtec.co.uk/
 *
 * SDHCI (HSMMC) support for infoTM 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>

#include <linux/mmc/host.h>

#include <plat/sdhci.h>
#include <plat/regs-sdhci.h>

#include "sdhci.h"

#define MAX_BUS_CLK	(2)

#ifdef CONFIG_SDI1_USE_EPLL_CLOCK
#define EPLL_CLK	1
#else
#define EPLL_CLK 	0
#endif

struct sdhci_imap {
	struct sdhci_host	*host;
	struct platform_device	*pdev;
	struct resource		*ioarea;
	struct imapx_sdi_platdata  *pdata;
	unsigned int		cur_clk;

	struct clk		*clk_io;
	struct clk		*clk_bus[MAX_BUS_CLK];
};
static void imapx_set_gpio(int channel,int width)
{
	switch(channel)
	{
		case 1:
			if(width == 4)
			{
				imapx_gpio_setcfg(IMAPX_GPO_RANGE(0, 5), IG_CTRL0, IG_NORMAL);
				imapx_gpio_pull(IMAPX_GPO_RANGE(0, 5), 0, IG_NORMAL);
			}
			printk(KERN_INFO "channel 1 doesn't work right now\n");
			break;

		default :
			printk(KERN_INFO "param error \n");
			break;
	}
}

static inline struct sdhci_imap *to_imap(struct sdhci_host *host)
{
	return sdhci_priv(host);
}

static void imapx_cfg_card(struct platform_device *dev,
				  struct sdhci_host *host)
{
	u32 ctrl2;
	struct sdhci_imap *ourhost = to_imap(host); 
	void __iomem *r = host->ioaddr;
	unsigned int clk_src;

	ctrl2 = readl(r + IMAP_SDHCI_CONTROL2);
	if(ourhost->cur_clk == 1) 
	{	
		ctrl2 |= IMAP_SDHCI_CTRL2_SELBASECLK_MASK;
	} else {
		ctrl2 &= ~IMAP_SDHCI_CTRL2_SELBASECLK_MASK;
	}

	ctrl2 |= (IMAP_SDHCI_CTRL2_ENSTAASYNCCLR |
		  IMAP_SDHCI_CTRL2_ENCMDCNFMSK |
		  IMAP_SDHCI_CTRL2_ENFBCLKRX |
		  IMAP_SDHCI_CTRL2_DFCNT_NONE |
		  IMAP_SDHCI_CTRL2_ENCLKOUTHOLD);

	writel(ctrl2, r + IMAP_SDHCI_CONTROL2);

	clk_src = readl(r + IMAP_SDHCI_CONTROL2);
}

				
/**
 * sdhci_imap_get_max_clk - callback to get maximum clock frequency.
 * @host: The SDHCI host instance.
 *
 * Callback to return the maximum clock rate acheivable by the controller.
*/
static unsigned int sdhci_imap_get_max_clk(struct sdhci_host *host)
{
	struct sdhci_imap *ourhost = to_imap(host);
	struct clk *busclk;
	unsigned int rate;

	busclk = ourhost->clk_io;
	rate = clk_get_rate(busclk);

	if(ourhost->cur_clk == 1)
		rate = rate / 5;

	
	return rate;
}

static unsigned int sdhci_imap_get_timeout_clk(struct sdhci_host *host)
{
	return sdhci_imap_get_max_clk(host) / 1000000;
}
static void sdhci_imap_set_clk_src(struct sdhci_host *host)
{
	struct sdhci_imap *ourhost = to_imap(host);
	unsigned int clk_src;
	clk_src = readl(host->ioaddr + IMAP_SDHCI_CONTROL2);
	
	if(ourhost->cur_clk == 1) 
	{	
		clk_src |= IMAP_SDHCI_CTRL2_SELBASECLK_MASK;
	} else {
		clk_src &= ~IMAP_SDHCI_CTRL2_SELBASECLK_MASK;
	}

	writel(clk_src,host->ioaddr + IMAP_SDHCI_CONTROL2);
	
}

static struct sdhci_ops sdhci_imap_ops = {
	.get_max_clock		= sdhci_imap_get_max_clk,
	.get_timeout_clock	= sdhci_imap_get_timeout_clk,
	.set_clk_src		= sdhci_imap_set_clk_src,
};


/******************************************************************/
static struct mmc_host *sdhci_host1 = NULL;

void sdhci_mmc1_detect_change(void)
{
		if(sdhci_host1)
			mmc_detect_change(sdhci_host1,msecs_to_jiffies(100));
}
EXPORT_SYMBOL(sdhci_mmc1_detect_change);
/*****************************************************************/



static int __devinit sdhci_imap_probe(struct platform_device *pdev)
{
	struct imapx_sdi_platdata *pdata = pdev->dev.platform_data;
	struct device *dev = &pdev->dev;
	struct sdhci_host *host;
	struct sdhci_imap *sc;
	struct resource *res;
	int ret, irq;
	unsigned int cfg_val;

	printk(KERN_INFO "[SDHCI1]:infoTM sdhci host driver init.\n");

	if (!pdata) {
		dev_err(dev, "no device data specified\n");
		return -ENOENT;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "no irq specified\n");
		return irq;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no memory specified\n");
		return -ENOENT;
	}

	host = sdhci_alloc_host(dev, sizeof(struct sdhci_imap));
	if (IS_ERR(host)) {
		dev_err(dev, "sdhci_alloc_host() failed\n");
		return PTR_ERR(host);
	}

	sc = sdhci_priv(host);

	sc->host = host;
	sc->pdev = pdev;
	sc->pdata = pdata;

	platform_set_drvdata(pdev, host);
	/* Using EPLL clocks */
	cfg_val = readl(rDIV_CFG3);
	cfg_val &= ~(0xff00);
	cfg_val |= 0x1200;
	writel(cfg_val, rDIV_CFG3);
	/*
	 * get clock source for SD: 0:HCLK,1:EPLL
	 */
	sc->cur_clk = EPLL_CLK;
	sc->clk_io = clk_get(dev,pdata->clocks[sc->cur_clk]);
	if (IS_ERR(sc->clk_io)) {
		dev_err(dev, "failed to get io clock\n");
		ret = PTR_ERR(sc->clk_io);
		goto err_io_clk;
	}

	clk_enable(sc->clk_io);
	sc->ioarea = request_mem_region(res->start, resource_size(res),
					mmc_hostname(host->mmc));
	if (!sc->ioarea) {
		dev_err(dev, "failed to reserve register area\n");
		ret = -ENXIO;
		goto err_no_busclks;
	}

	host->ioaddr = ioremap_nocache(res->start, resource_size(res));
	if (!host->ioaddr) {
		dev_err(dev, "failed to map registers\n");
		ret = -ENXIO;
		goto err_no_busclks;
	}

	imapx_set_gpio(pdata->hw_port,pdata->width);
	imapx_cfg_card(pdev,host);

	host->hw_name = "info_sdi1";
	host->ops = &sdhci_imap_ops;
	host->quirks = 0;
	host->irq = irq;

	/* Setup quirks for the controller */

	/* Currently with ADMA enabled we are getting some length
	 * interrupts that are not being dealt with, do disable
	 * ADMA until this is sorted out. */
#ifndef	CONFIG_MMC_SDHCI_IMAP_ADMA
	host->quirks |= SDHCI_QUIRK_BROKEN_ADMA;
	host->quirks |= SDHCI_QUIRK_32BIT_ADMA_SIZE;
#endif

#ifndef CONFIG_MMC_SDHCI_IMAP_SDMA

	/* we currently see overruns on errors, so disable the SDMA
	 * support as well. */
	host->quirks |= SDHCI_QUIRK_BROKEN_DMA;

	/* PIO currently has problems with multi-block IO */
	host->quirks |= SDHCI_QUIRK_NO_MULTIBLOCK;

#endif /* CONFIG_MMC_SDHCI_IMAP_SDMA */

	/* It seems we do not get an DATA transfer complete on non-busy
	 * transfers, not sure if this is a problem with this specific
	 * SDHCI block, or a missing configuration that needs to be set. */
	host->quirks |= SDHCI_QUIRK_NO_BUSY_IRQ;

	host->quirks |= (SDHCI_QUIRK_32BIT_DMA_ADDR |
			 SDHCI_QUIRK_32BIT_DMA_SIZE);

	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(dev, "sdhci_add_host() failed\n");
		goto err_add_host;
	}

	sdhci_host1 = host->mmc;

	return 0;

 err_add_host:
	release_resource(sc->ioarea);
	kfree(sc->ioarea);
  
 err_no_busclks:
	clk_disable(sc->clk_io);
	clk_put(sc->clk_io);

 err_io_clk:
	sdhci_free_host(host);


	return ret;
}

static int __devexit sdhci_imap_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_PM

static int sdhci_imap_suspend(struct platform_device *dev, pm_message_t pm)
{
	struct sdhci_host *host = platform_get_drvdata(dev);

	sdhci_suspend_host(host, pm);
	return 0;
}

static int sdhci_imap_resume(struct platform_device *dev)
{
	struct sdhci_host *host = platform_get_drvdata(dev);

	sdhci_resume_host(host);
	return 0;
}

#else
#define sdhci_imap_suspend NULL
#define sdhci_imap_resume NULL
#endif

static struct platform_driver sdhci_imap_driver = {
	.probe		= sdhci_imap_probe,
	.remove		= __devexit_p(sdhci_imap_remove),
	.suspend	= sdhci_imap_suspend,
	.resume	        = sdhci_imap_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "imapx200_sdi1",
	},
};

static int __init sdhci_imap_init(void)
{
	return platform_driver_register(&sdhci_imap_driver);
}

static void __exit sdhci_imap_exit(void)
{
	platform_driver_unregister(&sdhci_imap_driver);
}

module_init(sdhci_imap_init);
module_exit(sdhci_imap_exit);

MODULE_AUTHOR("Ben Dooks, <ben@simtec.co.uk>");
MODULE_LICENSE("GPL v2");
