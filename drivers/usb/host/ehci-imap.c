#include <linux/platform_device.h>
#include <linux/gpio.h>


#define IMAP_DEBUG	 0 

#if IMAP_DEBUG
#define IMAP_EHCI_DEBUG(debugs, ...)  printk(KERN_INFO debugs,  ##__VA_ARGS__)
#else
#define IMAP_EHCI_DEBUG(debugs, ...) do{}while(0)
#endif






static int ehci_imapx200_init(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int ret = 0;

	IMAP_EHCI_DEBUG("++ ehci_imapx200_init ++\n");
	

	ehci->caps = hcd->regs;
	IMAP_EHCI_DEBUG("hcd->regs is 0x%x hcs_params is 0x%x \r\n",hcd->regs,ehci_readl(ehci,&ehci->caps->hc_capbase));
	ehci->regs = hcd->regs + HC_LENGTH(ehci_readl(ehci,&ehci->caps->hc_capbase));
	ehci->hcs_params = ehci_readl(ehci,&ehci->caps->hcs_params);


	IMAP_EHCI_DEBUG("ehci->caps is 0x%x \r\n",ehci->caps);
	IMAP_EHCI_DEBUG("ehci->regs is 0x%x \r\n",ehci->regs);
	
	//hcd->has_tt = 1;
	//ehci->sbrn = 0x20;
	ret = ehci_halt(ehci);
	if (ret)
		return ret;


	ret = ehci_init(hcd);
	if(ret)
		return ret;

	ehci_reset(ehci);
	IMAP_EHCI_DEBUG("--ehci_imapx200_init \r\n");
	return ret;
}

static int usb_hcd_imapx200_probe(const struct hc_driver *driver,struct platform_device *pdev)
{
	struct platform_data *pInfo;
	struct usb_hcd *hcd;
	struct resource *r_mem;
	int irq, valTmp, i, ret = 0;
	
	IMAP_EHCI_DEBUG("++usb_hcd_imapx200_probe\r\n");
	pInfo = pdev->dev.platform_data;
	if(pInfo)
	{
		dev_err(&pdev->dev,"This Paramer <pdev> err !");
		return -ENODEV;
	}
	
	irq = platform_get_irq(pdev,0);
	if(irq < 0)
	{
		dev_err(&pdev->dev,"no resource of IORESOURCE_IRQ");
		return -ENXIO;
	}

	IMAP_EHCI_DEBUG("USB Resource IRQ is 0x%x \r\n",irq);
	
	r_mem = platform_get_resource(pdev,IORESOURCE_MEM,0);
	if(!r_mem)
	{
		dev_err(&pdev->dev,"no resource of IORESOURCE_MEM");
		return -ENXIO;
	}

	hcd = usb_create_hcd(driver, &pdev->dev,dev_name(&pdev->dev));
	if(!hcd)
	{
		dev_err(&pdev->dev,"usb_create_hcd failed!");
		return  -ENOMEM;
	}

	hcd->rsrc_start = r_mem->start;
	hcd->rsrc_len = r_mem->end - r_mem->start + 1;
	IMAP_EHCI_DEBUG("USB Resource MEM is 0x%x \r\n",r_mem->start);
	IMAP_EHCI_DEBUG("USB Resource len is 0x%x \r\n",hcd->rsrc_len);
	
	if(!request_mem_region(hcd->rsrc_start,hcd->rsrc_len,driver->description))
	{
		dev_err(&pdev->dev,"request_mem_region failed");
		ret = -EBUSY;
		goto err_put;
	}

	hcd->regs = ioremap(hcd->rsrc_start,hcd->rsrc_len);
	if(!hcd->regs)
	{
		dev_err(&pdev->dev,"ioremap_nocache failed");
		ret = -ENOMEM;
		goto err_ioremap;
	}

	IMAP_EHCI_DEBUG("USB Register Map Address 0x%x \r\n",hcd->regs);

	// config OTG to HOST
	 
	/* if not gadget, we initial otg as host */
#if defined(CONFIG_IG_OTG_HOST)
	valTmp = __raw_readl(rMEM_CFG);
	valTmp |= (1<<8);
	__raw_writel(valTmp,rMEM_CFG); 
	
	imapx_gpio_setcfg(IMAPX_GPE15, IG_OUTPUT, IG_NORMAL);
#endif	
	// epll setting
	valTmp = __raw_readl(rDIV_CFG2);
	valTmp &=~(3<<16);
	valTmp |= (2<<16);
	valTmp &=~(0x1f<<18);
	valTmp |=(8<<18);
	__raw_writel(valTmp,rDIV_CFG2); 
	
	// usb gate
	valTmp = __raw_readl(rPAD_CFG);
	valTmp &= ~0xe;
	__raw_writel(valTmp, rPAD_CFG);
	__raw_writel(0x0, rUSB_SRST);
	for(i=0;i<6000;i++);
	valTmp = __raw_readl(rPAD_CFG);
	valTmp |= 0xe;
	__raw_writel(valTmp,rPAD_CFG);
	mdelay(4);
	__raw_writel(0x5,rUSB_SRST);
	for(i=0;i<1000;i++);
	__raw_writel(0xf,rUSB_SRST);

	// OTG config



	/* if not gadget, we initial otg as host */
#if defined(CONFIG_IG_OTG_HOST)
	imapx_gpio_setpin(IMAPX_GPE15, 1, IG_NORMAL);
#endif
	dev_set_drvdata(&pdev->dev,hcd);	
	ret = usb_add_hcd(hcd,irq,IRQF_DISABLED);
	if(ret)
		goto err_add_hcd;

	IMAP_EHCI_DEBUG("--usb_hcd_imapx200_probe\r\n");

	return ret;

err_add_hcd:
	iounmap(hcd->regs);
err_ioremap:	
	release_mem_region(hcd->rsrc_start,hcd->rsrc_len);
err_put:
	usb_put_hcd(hcd);
	return ret;
}

static int usb_hcd_imapx200_remove(struct usb_hcd *hcd,struct platform_device *pdev)
{
	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start,hcd->rsrc_len);
	usb_put_hcd(hcd);

	return 0;
}

static const struct hc_driver ehci_imapx200_hc_driver = 
{
	.description = hcd_name,
	.product_desc = "Infotm Ehci Controller",
	.hcd_priv_size = sizeof(struct ehci_hcd),

	.irq = ehci_irq,
	.flags = HCD_USB2 | HCD_MEMORY,

	.reset = ehci_imapx200_init,
	.start = ehci_run,
	.stop = ehci_stop,
	.shutdown = ehci_shutdown,

	.urb_enqueue = ehci_urb_enqueue,
	.urb_dequeue = ehci_urb_dequeue,
	.endpoint_disable = ehci_endpoint_disable,
	.endpoint_reset = ehci_endpoint_reset,

	.get_frame_number = ehci_get_frame,

	.hub_status_data = ehci_hub_status_data,
	.hub_control = ehci_hub_control,
#if defined(CONFIG_PM)	
	.bus_suspend = ehci_bus_suspend,
	.bus_resume = ehci_bus_resume,
#endif	
	.relinquish_port = ehci_relinquish_port,
	.port_handed_over = ehci_port_handed_over,

	.clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,
};


static int ehci_imapx200_drv_probe(struct platform_device *pdev)
{
	if(usb_disabled())
		return -ENODEV;
	return usb_hcd_imapx200_probe(&ehci_imapx200_hc_driver,pdev);
}

static int ehci_imapx200_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_hcd_imapx200_remove(hcd,pdev);
	return 0;
}


#ifdef CONFIG_PM

static int ehci_hcd_imapx200_drv_suspend(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	unsigned long flags;
	int rc;

	rc = 0;

	if (time_before(jiffies, ehci->next_statechange))
		msleep(10);

	/* Root hub was already suspended. Disable irq emission and
	 * mark HW unaccessible, bail out if RH has been resumed. Use
	 * the spinlock to properly synchronize with possible pending
	 * RH suspend or resume activity.
	 *
	 * This is still racy as hcd->state is manipulated outside of
	 * any locks =P But that will be a different fix.
	 */
	spin_lock_irqsave(&ehci->lock, flags);
	if (hcd->state != HC_STATE_SUSPENDED) {
		rc = -EINVAL;
		goto bail;
	}
	ehci_writel(ehci, 0, &ehci->regs->intr_enable);
	(void)ehci_readl(ehci, &ehci->regs->intr_enable);

	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);


bail:
	spin_unlock_irqrestore(&ehci->lock, flags);

	// could save FLADJ in case of Vaux power loss
	// ... we'd only use it to handle clock skew

	return rc;

}

static int ehci_hcd_imapx200_drv_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);

	/****************************************************/
	int valTmp,i;
	valTmp = __raw_readl(rPAD_CFG);
	if((valTmp & 0xf) != 0xe)
	{
		printk(KERN_ERR "eeeee\n");
		valTmp = __raw_readl(rPAD_CFG);
		valTmp &= ~0xe;
		__raw_writel(valTmp, rPAD_CFG);
		__raw_writel(0x0, rUSB_SRST);
		for(i=0;i<6000;i++);
		valTmp = __raw_readl(rPAD_CFG);
		valTmp |= 0xe;
		__raw_writel(valTmp,rPAD_CFG);
		mdelay(4);
		__raw_writel(0x5,rUSB_SRST);
		for(i=0;i<6000;i++);
		__raw_writel(0xf,rUSB_SRST);
	}

	// maybe restore FLADJ

	if (time_before(jiffies, ehci->next_statechange))
		msleep(100);

	/* Mark hardware accessible again as we are out of D3 state by now */
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	/* If CF is still set, we maintained PCI Vaux power.
	 * Just undo the effect of ehci_pci_suspend().
	 */
	if (ehci_readl(ehci, &ehci->regs->configured_flag) == FLAG_CF) {
		int	mask = INTR_MASK;

		if (!hcd->self.root_hub->do_remote_wakeup)
			mask &= ~STS_PCD;
		ehci_writel(ehci, mask, &ehci->regs->intr_enable);
		ehci_readl(ehci, &ehci->regs->intr_enable);
		return 0;
	}

	ehci_dbg(ehci, "lost power, restarting\n");
	usb_root_hub_lost_power(hcd->self.root_hub);

	/* Else reset, to cope with power loss or flush-to-storage
	 * style "resume" having let BIOS kick in during reboot.
	 */
	(void) ehci_halt(ehci);
	(void) ehci_reset(ehci);

	/* emptying the schedule aborts any urbs */
	spin_lock_irq(&ehci->lock);
	if (ehci->reclaim)
		end_unlink_async(ehci);
	ehci_work(ehci);
	spin_unlock_irq(&ehci->lock);

	ehci_writel(ehci, ehci->command, &ehci->regs->command);
	ehci_writel(ehci, FLAG_CF, &ehci->regs->configured_flag);
	ehci_readl(ehci, &ehci->regs->command);	/* unblock posted writes */

	/* here we "know" root ports should always stay powered */
	ehci_port_power(ehci, 1);

	hcd->state = HC_STATE_SUSPENDED;

	return 0;

}	

static struct dev_pm_ops imapx200_ehci_pmops = {
	.suspend = ehci_hcd_imapx200_drv_suspend,
	.resume = ehci_hcd_imapx200_drv_resume,
};

#define IMAPX200_EHCI_PMOPS &imapx200_ehci_pmops

#else
#define IMAPX200_EHCI_PMOPS NULL
#endif


MODULE_ALIAS("platform:imapx200_usbhost20");

static struct platform_driver ehci_imapx200_driver = 
{
	.probe = ehci_imapx200_drv_probe,
	.remove = ehci_imapx200_drv_remove,
	.shutdown = usb_hcd_platform_shutdown,
	.driver = {
		.name = "imapx200_usbhost20",
		.owner	= THIS_MODULE,
		.pm	= IMAPX200_EHCI_PMOPS,
	},
};
