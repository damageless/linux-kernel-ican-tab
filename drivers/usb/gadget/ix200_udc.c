/***************************************************************************** 
** drivers/usb/gadget/ix200_udc.c
** 
** Copyright (c) 2009~2014 ShangHai Infotm Ltd all rights reserved. 
** 
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
** 
** Description: PCB test, module .
**
** Author:
**      warits <warits.wang@infotmic.com.cn>
**      
** Revision History: 
** ----------------- 
** 0  XXX 06/30/2010 XXX	
*****************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/gpio.h>

#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <linux/usb.h>
#include <linux/usb/gadget.h>

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <mach/irqs.h>

#include <mach/hardware.h>
#include "ix200_udc.h"



static uint32_t g_rx_flag;

static void __ep_tx(int ep_num);


#define THREAD_INITIAL	1
#define THREAD_START	2
#define THREAD_STOP		3
#define THREAD_DEAD		4
#define THREAD_EN		5
#define THREAD_DIS		6

#define USB_BUS			(0xff)
static int thread_state = 0;
struct timer_list otg_timer;

// otg thread wait queue
DECLARE_WAIT_QUEUE_HEAD(otg_queue);
#define sleep_thread()   interruptible_sleep_on(&otg_queue)
#define wakeup_thread()  wake_up_interruptible(&otg_queue)

DECLARE_WAIT_QUEUE_HEAD(timer_queue);

/* FIXME: Need extern in .h */


static const struct usb_ep_ops ix_ep_ops = {
	.enable			= ix_ep_enable,
	.disable		= ix_ep_disable,
	.alloc_request	= ix_alloc_request,
	.free_request	= ix_free_request,
	.queue			= ix_queue,
	.dequeue		= ix_dequeue,
	.set_halt		= ix_set_halt,
};

static const struct usb_gadget_ops ix_gadget_ops = {
	.get_frame			= ix_get_frame,
	.wakeup				= ix_wakeup,
	.set_selfpowered	= ix_set_selfpowered,
	.pullup				= ix_pullup,
	.vbus_session		= ix_vbus_session,
	.vbus_draw			= ix_vbus_draw,
};

/* the iMAPx200 UDC */
static struct ix200_udc ix_udc = {
	.gadget = {
		.is_otg		= 0,
		.ops		= &ix_gadget_ops,
		.ep0		= &ix_udc.ep[0],
		.name		= "iMAPx200-UDC",
		.dev		= {
			.init_name = "gadget",
		},
	},

	/* ep0 */
	.ep[0].name = "ep0",
	.ep[0].maxpacket = EP0_MAXPACKET,
	.ep[1].name = "ep1",
	.ep[1].maxpacket = EP_MAXPACKET,
	.ep[2].name = "ep2",
	.ep[2].maxpacket = EP_MAXPACKET,
	.ep[3].name = "ep3",
	.ep[3].maxpacket = EP_MAXPACKET,
	.ep[4].name = "ep4",
	.ep[4].maxpacket = EP_MAXPACKET,

	/* due to TBCR configurations,
	 * ep1 has 2x512b, ep3 has 1x512b
	 */
	.ep_bufcount[1] = 3,
	.ep_bufcount[3] = 1,

	.devstat = 0,
};

static uint32_t __b_connect(void)
{
	uint32_t ret = 0;
	uint32_t i =0;

	if((readl(__reg + USB_BCSR) & USB_B_Valid))
	{
		dprintk(1, "b device votage OK\n");
		writel(readl(__reg + USB_BCWR) | USB_B_Connect,__reg + USB_BCWR);
		while((readl(__reg + USB_BCSR) & USB_DRDB_CS) != 0x10)
		{
			i++;
			if(i<60000) //timer out
			{
				msleep(1);
			}
			else
			{
				break;
			}
		}
		dprintk(1, "connected with host\n");
		writel(readl(__reg + USB_BCWR) & ~USB_B_Connect,__reg + USB_BCWR);

		writel(0x00000000, __reg + USB_TBCR0);
		writel(0x00000000, __reg + USB_TBCR1);
		writel(0x00000180, __reg + USB_TBCR2);
		writel(0x00000180, __reg + USB_TBCR3);

		writel(0x00000000, __reg + USB_ACR);
		writel(0xffffffff, __reg + USB_IDR);
		writel(0x0007001f, __reg + USB_IER);

		writel(0x0000003f,__reg + USB_BCIDR);
		writel(0x00000008,__reg + USB_BCIER);
		writel(0x00000008,__reg + USB_BCISR);

		writel(0x0, __reg + USB_FNCR);
		writel(0x00070001,__reg + USB_UDCR);
		writel(0x1, __reg + USB_BFCR);
		ret = 1;
	}

	return ret;
}

void otg_timer_func(unsigned long foo)
{
	wakeup_thread();
}

#ifdef CONFIG_IG_OTG_AUTO
static uint8_t  __start_check;
static uint8_t  __is_connect;
#endif

int monitor_connect_thread(void *foo)
{
#ifdef CONFIG_IG_OTG_AUTO
	uint32_t __tt;
	uint8_t  __id_high;
	uint32_t valTmp, i, ovbus, oid;


	__is_connect = 0;
	__start_check = 1;
#endif

	init_timer(&otg_timer);
	otg_timer.function = otg_timer_func;


#ifdef CONFIG_IG_OTG_AUTO
	ovbus = __imapx_name_to_gpio(CONFIG_IG_OTG_VBUS);
	oid = __imapx_name_to_gpio(CONFIG_IG_OTG_ID);

	if(ovbus != IMAPX_GPIO_ERROR)
	  imapx_gpio_setcfg(ovbus, IG_OUTPUT, IG_NORMAL);

	if(oid != IMAPX_GPIO_ERROR) {
		imapx_gpio_setcfg(oid, IG_INPUT, IG_NORMAL);
		imapx_gpio_pull(oid, 1, IG_NORMAL);
	}

	msleep(500);
	
	__id_high = !imapx_gpio_getpin(oid, IG_NORMAL);
	printk(KERN_ERR " ------ monitor_connect_thread --------");
#endif

	while(thread_state != THREAD_STOP)
	{
		if(thread_state == THREAD_DIS)
		{
#ifndef CONFIG_IG_OTG_AUTO 			
			if(__b_connect())
				thread_state = THREAD_EN;
#else
			//GPL5
			if(__id_high == 1)
			{
				if(__start_check)
				{
					if(__is_connect == 0)
					{
						if(__b_connect())
						{
							__is_connect = 1;
							__start_check = 0;
							printk(KERN_ERR "~~~~~ device connect ~~~~\n");
						}
					}
				}


				__tt = imapx_gpio_getpin(oid, IG_NORMAL);
				if(!__tt)
				{
					__id_high = 0;
					// usb host running
	
					valTmp = __raw_readl(rMEM_CFG);
					valTmp |= (1<<8);
					__raw_writel(valTmp,rMEM_CFG);

					imapx_gpio_setpin(ovbus, 1, IG_NORMAL);

					// usb gate
					valTmp = __raw_readl(rPAD_CFG);
					valTmp &= ~0x8;
					__raw_writel(valTmp, rPAD_CFG);

					__raw_writel(0xc, rUSB_SRST);
					for(i=0;i<6000;i++);
					valTmp = __raw_readl(rPAD_CFG);
					valTmp |= 0x8;
					__raw_writel(valTmp,rPAD_CFG);
					mdelay(4);
					__raw_writel(0xd,rUSB_SRST);
					for(i=0;i<1000;i++);
					__raw_writel(0xf,rUSB_SRST);

					//__ehci_bus_resume();
					//__ohci_bus_resume();
					msleep(500);
					__start_check = 0;
					printk(KERN_ERR "+++++++ id high ++++++++\n");
				
				}
			}
			else
			{
				__tt = imapx_gpio_getpin(oid, IG_NORMAL);
				if(__tt)
				{
					__id_high = 1;
					printk(KERN_ERR "++++++ id low ++++++++\n");
					//__ehci_bus_suspend();
					//__ohci_bus_suspend();
					//msleep(1000);

					valTmp = __raw_readl(rMEM_CFG);
					valTmp &= ~(1<<8);
					__raw_writel(valTmp,rMEM_CFG);

					imapx_gpio_setpin(ovbus, 0, IG_NORMAL);

					// usb gate
					valTmp = __raw_readl(rPAD_CFG);
					valTmp &= ~0x8;
					__raw_writel(valTmp, rPAD_CFG);

					__raw_writel(0xc, rUSB_SRST);
					for(i=0;i<6000;i++);
					valTmp = __raw_readl(rPAD_CFG);
					valTmp |= 0x8;
					__raw_writel(valTmp,rPAD_CFG);
					mdelay(4);
					__raw_writel(0xd,rUSB_SRST);
					for(i=0;i<1000;i++);
					__raw_writel(0xf,rUSB_SRST);

					__start_check = 1;
					__is_connect = 0;
					msleep(500);
					printk(KERN_ERR "------ id low -------\n");

				}
			}
#endif			

		}
		mod_timer(&otg_timer,jiffies + HZ/2);
		sleep_thread();
	}
	
	del_timer(&otg_timer);
	thread_state = THREAD_DEAD;

	return 0;
}

void start_monitor_thread()
{
	kernel_thread(monitor_connect_thread, (void *)NULL, CLONE_FS | CLONE_FILES | CLONE_SIGHAND);
}

void stop_monitor_thread()
{
	thread_state = THREAD_STOP;
	wakeup_thread();
	while(thread_state != THREAD_DEAD)
		yield();
}


int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	int ret;
	dprintk(DEBUG_NORMAL, "usb_gadget_register_driver() '%s'\n",
		driver->driver.name);

	if(ix_udc.driver)
	  return -EBUSY;

	if (!driver->bind || !driver->setup
			|| driver->speed < USB_SPEED_FULL) {
		printk(KERN_ERR "Invalid driver: bind %p setup %p speed %d\n",
			driver->bind, driver->setup, driver->speed);
		return -EINVAL;
	}
#if defined(MODULE)
	if (!driver->unbind) {
		printk(KERN_ERR "Invalid driver: no unbind method\n");
		return -EINVAL;
	}
#endif

	/* Hook the driver */
	ix_udc.driver = driver;
	ix_udc.gadget.dev.driver = &driver->driver;

	/* Bind the driver */
	if ((ret = device_add(&ix_udc.gadget.dev)) != 0) {
		printk(KERN_ERR "Error in device_add() : %d\n",ret);
		goto register_error;
	}

	dprintk(DEBUG_NORMAL, "binding gadget driver '%s'\n",
		driver->driver.name);

	dprintk(1, "ep0max=%x\n", ix_udc.ep[0].maxpacket);
	if ((ret = driver->bind (&ix_udc.gadget)) != 0) {
		device_del(&ix_udc.gadget.dev);
		goto register_error;
	}

	/* Enable UDC */
	ix_udc_en(1);

	thread_state = THREAD_DIS;
	start_monitor_thread();
	return 0;

register_error:
	ix_udc.driver = NULL;
	ix_udc.gadget.dev.driver = NULL;
	return ret;
}

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	if (!driver || driver != ix_udc.driver || !driver->unbind)
		return -EINVAL;

	dprintk(DEBUG_NORMAL,"usb_gadget_unregister_driver() '%s'\n",
		driver->driver.name);

	driver->unbind(&ix_udc.gadget);
	device_del(&ix_udc.gadget.dev);
	ix_udc.driver = NULL;

	/* Disable UDC */
	ix_udc_en(0);

	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);
EXPORT_SYMBOL(usb_gadget_register_driver);


/* the implement */
static inline struct list_head *
ix_find_queue(struct usb_ep *ep)
{
	struct ix200_udc *udc = &ix_udc;
	int i;
	for(i = 0; i < IX_EP_COUNT; i++)
	  if(ep == &udc->ep[i])
		return &udc->ep_req[i];

	return NULL;
}

static int ix_get_ep_num(struct usb_ep *ep)
{
	int i;

	for (i = 0; i < IX_EP_COUNT; i++)
	  if(&ix_udc.ep[i] == ep)
		break;

	return i;
}

static inline struct ix200_request *to_ix200_req(struct usb_request *req)
{
	return container_of(req,struct ix200_request,req);
}


static inline void ix_ep_intr_enable(int ep_num)
{
	writel(1<<(23+ep_num),__reg+USB_IER);
}

static inline void ix_ep_intr_disable(int ep_num)
{
	writel(1<<(23+ep_num),__reg+USB_IDR);
}

static int ix_ep_enable(struct usb_ep *ep, 
   const struct usb_endpoint_descriptor *desc)
{
	uint32_t max, edr = 0;
	unsigned long flags;
	int ep_num = ix_get_ep_num(ep);

	dprintk(4, "udc: %s, ep%d\n", __func__, ep_num);
	if (!ep || !desc /* || ep->desc */
			/* || ep->name == "ep0" */
			|| desc->bDescriptorType != USB_DT_ENDPOINT)
		return -EINVAL;

	if(!ix_udc.driver || ix_udc.gadget.speed == USB_SPEED_UNKNOWN)
	  return -ESHUTDOWN;

	max = desc->wMaxPacketSize & 0x1fff;
	local_irq_save(flags);
	ep->maxpacket = max & 0x7ff;

	dprintk(4, "maxpacket:%x\n", max );
	ix_udc.desc[ep_num] = desc;

	/* write maxpacket value to hw */
	if(!ep_num)
	  writel(readl(__reg + USB_IER) | USB_EP0,
		 __reg + USB_IER);
	else
	{
		if(desc->bEndpointAddress & USB_DIR_IN)
		{
			edr |= USB_EP_DirIn;
			edr |= USB_InBuf(ep_num);		/* select TBCR1 */
		}

		dprintk(4, "DescriptorType %x\n", desc->bDescriptorType);
		switch(desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
		{
			case USB_ENDPOINT_XFER_BULK:
				edr |= USB_EP_Type(0x1);
				break;
			case USB_ENDPOINT_XFER_ISOC:
				edr |= USB_EP_Type(0x4);
				break;
			case USB_ENDPOINT_XFER_INT:
				edr |= USB_EP_Type(0x2);
				break;
		}

		edr |= USB_LogicNo(ep_num);		/* set logic number */
//		edr |= USB_EP_Phy(ep_num);		/* set logic number */
//		edr |= USB_EP_Alt(ep_num);		/* set logic number */
		/* FIXME: some interface values is not set */

	}

	/* set maxpacket */
	edr |= USB_MaxPacket(ep->maxpacket);

	dprintk(1, "write desc to ep%d, desc=%x\n", ep_num, edr);
	/* write desc */
	writel(edr, __reg + USB_EDR(ep_num));

	local_irq_restore(flags);
	return 0;
}

static int ix_ep_disable(struct usb_ep *ep)
{
	unsigned long flags;

	if (!ep /* || !ep->desc */) {
		dprintk(DEBUG_NORMAL, "%s not enabled\n",
			ep ? ep->name : NULL);
		return -EINVAL;
	}

	local_irq_save(flags);

	if(ep == &ix_udc.ep[0]) {
		/* clear all ep0 status bits */
		writel(readl(__reg + USB_BFCR) | USB_Flush_TXB(0),
		   __reg + USB_BFCR);
	} else {
		writel(readl(__reg + USB_BFCR) | USB_Flush_TXB(1),
		   __reg + USB_BFCR);
	}

	dprintk(4, "ep_disable: %s\n", ep->name);
	__req_nuke(ep, -ESHUTDOWN);

	local_irq_restore(flags);

	return 0;
}

static struct usb_request *
ix_alloc_request(struct usb_ep *ep, gfp_t mem_flags)
{
	struct ix200_request *req;

	dprintk(DEBUG_VERBOSE,"%s(%p,%d)\n", __func__, ep, mem_flags);

	if(!ep)
	  return NULL;

	req = kzalloc(sizeof(struct ix200_request), mem_flags);
	if(!req)
	  return NULL;
	
	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

static void
ix_free_request(struct usb_ep *ep, struct usb_request *req)
{
	struct ix200_request *_req = to_ix200_req(req);

	dprintk(DEBUG_VERBOSE, "%s(%p,%p)\n", __func__, ep, req);

	if (!ep || !req)
	  return ;
	
	WARN_ON(!list_empty(&_req->queue));
	kfree(_req);
}

static int ix_queue(struct usb_ep *ep, struct usb_request *req,
   gfp_t gfp_flags)
{
	unsigned long flags;
	struct ix200_request *_req = to_ix200_req(req);
	int ep_num = ix_get_ep_num(ep);
	const struct usb_endpoint_descriptor *desc =
		ix_udc.desc[ep_num];

	dprintk(0, "s, ep= %d %d\n", ep_num,req->length);
	if(unlikely(!ep)) {
		dprintk(DEBUG_NORMAL, "%s: invalid args\n", __func__);
		return -EINVAL;
	}

	if(unlikely(!ix_udc.driver
		   || ix_udc.gadget.speed == USB_SPEED_UNKNOWN)) {
		return -ESHUTDOWN;
	}

	local_irq_save(flags);

	if (unlikely(!req || !req->complete
			|| !req->buf /*|| !list_empty(&_req->queue)*/)) {
		if (!req)
			dprintk(DEBUG_NORMAL, "%s: 1 X X X\n", __func__);
		else {
			dprintk(DEBUG_NORMAL, "%s: 0 %01d %01d %01d\n",
				__func__, !req->complete,!req->buf, 
				!list_empty(&_req->queue));
		}

		local_irq_restore(flags);
		return -EINVAL;
	}

	req->status = -EINPROGRESS;
	req->actual = 0;

	/* pio or dma irq handler advances the queue. */
	if(likely(_req))
	{
		struct list_head *queue = ix_find_queue(ep);
		if(queue)
			list_add_tail(&_req->queue, queue);
		else
		{
			printk(KERN_ERR "Can not find queue for ep %p\n", ep);
			return -EINVAL;
		}
		/* kickstart this i/o queue? */
		if(ep_num)
		{
			if((desc->bEndpointAddress & USB_DIR_IN)!=0)
			{
				__ep_tx(ep_num);
			}
			else
			{
				g_rx_flag++;
				ix_ep_intr_enable(ep_num);
			}
		}
	}

	dprintk(0, "e\n" );
	local_irq_restore(flags);

	return 0;
}

static int ix_dequeue(struct usb_ep *ep, struct usb_request *req)
{
	struct ix200_request *_req= NULL;
	struct list_head *queue = ix_find_queue(ep);
	unsigned long flags;
	int ret;

	dprintk(DEBUG_VERBOSE, "%s(%p,%p)\n", __func__, ep, req);
	dprintk(1, "ix_dequeue: %s\n", __func__);

	if(!queue)
	{
		printk(KERN_ERR "Can not find queue for ep %p\n", ep);
		return -EINVAL;
	}

	if(!ix_udc.driver)
	  return -ESHUTDOWN;

	if(!ep || !req)
	  return -EINVAL;

	local_irq_save(flags);

	list_for_each_entry(_req, ix_find_queue(ep), queue)
	{
		if(&_req->req == req) {
			list_del_init(&_req->queue);
			req->status = -ECONNRESET;
			ret = 0;
			break;
		}
	}

	if(!ret) {
		dprintk(DEBUG_VERBOSE,
			"dequeued req %p from %s, len %d buf %p\n",
			_req, ep->name, req->length, req->buf);

		__req_done(ep, _req, -ECONNRESET); //TODO
	}

	local_irq_restore(flags);
	return ret;
}

static int ix_set_halt(struct usb_ep *ep, int value)
{
	unsigned long flags;
	int ep_num = ix_get_ep_num(ep);

	dprintk(1, "udc: %s\n", __func__);
	if(unlikely(!ep)) {
		dprintk(DEBUG_NORMAL, "%s: inval 2\n", __func__);
		return -EINVAL;
	}

	local_irq_save(flags);

	writel(readl(__reg + USB_FHHR) | USB_PEP_HALT(ep_num),
	   __reg + USB_FHHR);
	writel(0x1,__reg+USB_BFCR);
	local_irq_restore(flags);
	return 0;
}

/* gadget operations */
static int ix_get_frame(struct usb_gadget *gadget)
{
	dprintk(DEBUG_VERBOSE, "%s()\n", __func__);

	return readl(__reg + USB_FNCR) & 0x7ff;
}

static int ix_wakeup(struct usb_gadget *gadget)
{
	dprintk(DEBUG_NORMAL, "%s()\n", __func__);
	/* Not implemented in s3c */
	return 0;
}

static int ix_set_selfpowered(struct usb_gadget *gadget, int value)
{
	dprintk(DEBUG_NORMAL, "%s()\n", __func__);

	if(value)
	  ix_udc.devstat |=  (1 << USB_DEVICE_SELF_POWERED);
	else
	  ix_udc.devstat &= ~(1 << USB_DEVICE_SELF_POWERED);

	return 0;
}

static int ix_pullup(struct usb_gadget *gadget, int is_on)
{
	dprintk(DEBUG_NORMAL, "%s()\n", __func__);
	dprintk(1, "%s: find otg pull up\n", __func__);
	return 0;
}

static int ix_vbus_session(struct usb_gadget *gadget, int is_active)
{
	dprintk(DEBUG_NORMAL, "%s()\n", __func__);
	dprintk(1, "#### %s() ###\n", __func__);

	ix_udc.vbus = !!is_active;
	ix_pullup(gadget, !is_active);
	return 0;
}

static int ix_vbus_draw(struct usb_gadget *gadget, unsigned ma)
{
	dprintk(1, "#### udc: %s ####\n", __func__);
	return -ENOTSUPP;
}

static void __config_setup(void)
{
	struct usb_ctrlrequest crq;
	int ret;

	dprintk(2, "%s()\n", __func__);

	crq.wValue = 0x01;
	crq.bRequest = 0x09;
	crq.bRequestType = 0x00;
	crq.wLength = 0x00;
	crq.wIndex = 0x00;

	ix_udc.req_config = crq.wValue;
	ret = ix_udc.driver->setup(&ix_udc.gadget, &crq);
	if(ret < 0){
		if(ix_udc.req_config) {
			dprintk(DEBUG_NORMAL, "config change %02x fail %d?\n",
			   crq.bRequest, ret);
			return ;
		}

		if(ret == -EOPNOTSUPP)
			dprintk(DEBUG_NORMAL, "udc: Operation not supported\n");
		else
			dprintk(DEBUG_NORMAL,
				"dev->driver->setup failed. (%d)\n", ret);
		printk(KERN_ERR "_____config_step______\n");
		/* XXX */
		writel(USB_PEP_HALT(0), __reg + USB_FHHR);
		writel(USB_Flush_All, __reg + USB_BFCR);
	} else if (ix_udc.req_pending) {
		dprintk(0, "dev->req_pending... what now?\n");
		ix_udc.req_pending=0;
	}
	return ;
}

static void __address_setup(u32 val)
{
	struct usb_ctrlrequest crq;
	int ret;

	dprintk(2, "%s()\n", __func__);

	crq.wValue = val;
	crq.bRequest = 0x05;
	crq.bRequestType = 0x00;
	crq.wLength = 0x00;
	crq.wIndex = 0x00;

	ret = ix_udc.driver->setup(&ix_udc.gadget, &crq);
	if(ret < 0){
		if(ix_udc.req_config) {
			dprintk(DEBUG_NORMAL, "config change %02x fail %d?\n",
			   crq.bRequest, ret);
			return ;
		}

		if(ret == -EOPNOTSUPP)
			dprintk(DEBUG_NORMAL, "udc: Operation not supported\n");
		else
			dprintk(DEBUG_NORMAL,
				"dev->driver->setup failed. (%d)\n", ret);

		/* XXX */
		writel(USB_PEP_HALT(0), __reg + USB_FHHR);
		writel(USB_Flush_All, __reg + USB_BFCR);
	} else if (ix_udc.req_pending) {
		dprintk(0, "dev->req_pending... what now?\n");
		ix_udc.req_pending=0;
	}

	return ;
}
/* interrupt handle */
static void __ep0_setup(void)
{
	struct usb_ctrlrequest crq;
	int ret;

	dprintk(0, "%s()\n", __func__);

	if(ix_udc.gadget.speed == USB_SPEED_UNKNOWN)
	{
		if(readl(__reg + USB_UDCR) & USB_DSI)
		{
			dprintk(1,"usb speed high\n");
			ix_udc.gadget.speed = USB_SPEED_HIGH;
		}
		else
		{
			dprintk(1,"usb speed full\n");
			ix_udc.gadget.speed = USB_SPEED_FULL;
		}
	}
	/* read crq from reg */                               
	*(((uint32_t *)&crq) + 0) = readl(__reg + USB_STR0);  
	*(((uint32_t *)&crq) + 1) = readl(__reg + USB_STR1);  

	dprintk(2, "r:0x%xrt:0x%xv:0x%xl:0x%xi:0x%x\n", crq.bRequest, crq.bRequestType,
	   crq.wValue, crq.wLength, crq.wIndex);

	/* cope with automagic for some standard requests */
	ix_udc.req_std = (crq.bRequestType & USB_TYPE_MASK)
		== USB_TYPE_STANDARD;
	ix_udc.req_config = 0;
	ix_udc.req_pending = 1;

	ret = ix_udc.driver->setup(&ix_udc.gadget, &crq);
	udelay(5);
	return ;
}

static void __req_done(struct usb_ep *ep,
   struct ix200_request *req, int stat)
{
	list_del_init(&req->queue);

	if (likely (req->req.status == -EINPROGRESS))
	  req->req.status = stat;
	else
		stat = req->req.status;

	req->req.complete(ep, &req->req);
}

static void __req_nuke(struct usb_ep *ep, int stat)
{
	struct list_head *list = ix_find_queue(ep);

	if(!list)
	  return ;

	while (!list_empty (list)) {
		struct ix200_request *req;
		req = list_entry (list->next, struct ix200_request,
		   queue);
		__req_done(ep, req, stat);
	}
}

static void __ep_rx(int ep_num)
{
	struct usb_ep *ep = &ix_udc.ep[ep_num];
	struct list_head *list = ix_find_queue(ep);
	struct ix200_request *req;
	uint8_t *buf;
	uint32_t size, acr, max_fifo;
	uint32_t i;

	dprintk(0, "r,ep:%d\n", ep_num);
	if(unlikely(list_empty(list)))
	  req = NULL;
	else
	  req = list_entry(list->next, struct ix200_request, queue);

	if(!req)
	{
		dprintk(1, "Rx: no request in queue\n");
		return ;
	}
	/* begin transfer */
	max_fifo = readl(__reg + USB_PRIR) & USB_ReqCnt_MSK;
	buf = req->req.buf + req->req.actual;
	size = min(req->req.length - req->req.actual, max_fifo);
	req->req.dma = dma_map_single(NULL, (void *)d_buffer, size, DMA_FROM_DEVICE);

	dprintk(0, "Rx, size=%x, max=%x, ep=%d, PRIR=%08x\n",
	   size, max_fifo, ep_num, readl(__reg + USB_PRIR));
	writel(req->req.dma, __reg + USB_MDAR);
	writel(USB_ReqLength(size) | USB_IN_Prebuffer | USB_DMA_RxEn | USB_QueryACK, __reg + USB_ACR);
	for(i=0;i<10;i++);
	/* wait DMA finish */
	while(1) {
		acr = readl(__reg + USB_ACR);
		if(acr & USB_ReqError)
		{
			dprintk(1, "udc detect: rx dma error.\n");
			/* clear error state */
			writel(acr | USB_ReqError, __reg + USB_ACR);
			writel(readl(__reg + USB_FHHR) |
			   USB_PEP_HALT(ep_num), __reg + USB_FHHR);
			writel(readl(__reg + USB_BFCR) | 0x1, __reg + USB_BFCR);
			goto __exit_rx__;
		}
	   	else if(!(acr & 0x830))
		{
			dprintk(0, "udc detect: dma finished.\n");
			break;
		}
	}

	writel(USB_PEP_Tran(ep_num), __reg + USB_ISR);
	memcpy(buf, d_buffer, size);
	req->req.actual += size;

#if 0 
	{
		int i,j;
		char descx[1024];
		j=size>32?32:size;
		for (i = 0; i < j; i++)
		  sprintf(descx + 3 * i, "%02x ", d_buffer[i]);

		dprintk(2, "desc->[%s]\n", descx);
	}
#endif

	/* transfer finished */
	if((size != ep->maxpacket) || (req->req.actual == req->req.length))
	{
		g_rx_flag--;
		if(!g_rx_flag)
	  		ix_ep_intr_disable(ep_num);
 		__req_done(ep, req, 0);
 		dprintk(0, "rc %d %d",g_rx_flag,size);
	}

__exit_rx__:
	dma_unmap_single(NULL, req->req.dma, size, DMA_FROM_DEVICE);
	return ;
}

static void __ep_tx(int ep_num)
{
	struct usb_ep *ep = &ix_udc.ep[ep_num];
	//const struct usb_endpoint_descriptor *desc = ix_udc.desc[ep_num];
	struct list_head *list = ix_find_queue(ep);
	struct ix200_request *req;
	uint8_t *buf;
	uint32_t size, acr;
	uint32_t i;
	uint32_t fCompleted = 0;

	dprintk(0, "t,e%d\n",ep_num);
	/* get ep status */
	if(readl(__reg + USB_EDR(ep_num)) & USB_EP_HALT)
	{
		dprintk(1, "udc: ep%d halted. nuke queue.\n", ep_num);
		__req_nuke(ep, 0);
	}

	if(unlikely(list_empty(list)))
	  req = NULL;
	else
	  req = list_entry(list->next, struct ix200_request, queue);

	if(!req)
	{
		dprintk(1, "Tx: no request in queue\n");
		return ;
	}

	while(1)
	{
		/* begin transfer */
		buf = req->req.buf + req->req.actual;
		size = min((uint32_t)(req->req.length - req->req.actual), (ep_num?EP_MAXPACKET:EP0_MAXPACKET));
		memcpy(d_buffer, buf, size);

#if 0 
		{
			int i,j;
			char descx[1024];
			j=size>32?32:size;
			for (i = 0; i < j; i++)
			  sprintf(descx + 3 * i, "%02x ", d_buffer[i]);

			dprintk(2, "desc->[%s]\n", descx);
		}
#endif
		req->req.dma = dma_map_single(NULL, (void *)d_buffer, size, DMA_TO_DEVICE);

		writel(req->req.dma, __reg + USB_MDAR);

		acr = USB_ReqLength(size) | USB_QueryACK | USB_IN_Prebuffer | USB_DMA_TxEn | USB_TxBuffer(ep_num);
		writel(acr, __reg + USB_ACR);

		for(i=0;i<10;i++);
		/* wait DMA finish */
		while(1) {
			acr = readl(__reg + USB_ACR);
			if(acr & USB_ReqError)
			{
				dprintk(1, "udc detect: dma error.\n");
				/* clear error state */
				writel(acr | USB_ReqError, __reg + USB_ACR);
				writel(readl(__reg + USB_FHHR) | USB_PEP_HALT(ep_num), __reg + USB_FHHR);
				writel(readl(__reg + USB_BFCR) | 0x1, __reg + USB_BFCR);
				goto __exit_tx__;
			}
		   	else if(!(acr & 0x830))
			{
				dprintk(0, "udc detect: dma finished.\n");
				break;
			}
		}
		req->req.actual += size;
		
		i=0;
		while(1)
		{
//			if((readl(__reg + USB_TBCR(ep_num)) & 0x7000))
			if((readl(__reg + USB_TBCR(ep_num)) & 0x7000)
			   >= (ix_udc.ep_bufcount[ep_num] << 12))
			{
				i++;
				udelay(5);
			}
			else
				break;
//			if(i>60000) //timer out
//				break;
		}

		if(req->req.actual == req->req.length)
		{
			fCompleted = 1;
			break;
		}
	}

	i = 0;
	while(1)
	{
		if(readl(__reg + USB_TBCR(ep_num)) & 0x7000)
		{
			i++;
			udelay(5);
		}
		else
			break;
		if(i>60000) //timer out
			break;
	}

	/* transfer finished */
	//if((req->req.zero && !size) ||(size != ep->maxpacket) ||(!req->req.zero && (req->req.actual == req->req.length)))
	if(fCompleted)
	{
		__req_done(ep, req, 0);
		dprintk(0, "tc:%d %d %d\n",size,req->req.actual,req->req.length);
	}

__exit_tx__:
	dma_unmap_single(NULL, req->req.dma, size, DMA_TO_DEVICE);
	return ;
}

static void __ep_tx_post(int ep_num)
{
	struct usb_ep *ep = &ix_udc.ep[ep_num];
	//const struct usb_endpoint_descriptor *desc = ix_udc.desc[ep_num];
	struct list_head *list = ix_find_queue(ep);
	struct ix200_request *req;
	uint8_t *buf;
	uint32_t size, acr;
	uint32_t i;

	dprintk(0, "es:%d\n",ep_num);
	/* get ep status */
	if(readl(__reg + USB_EDR(ep_num)) & USB_EP_HALT)
	{
		dprintk(1, "udc: ep%d halted. nuke queue.\n", ep_num);
		__req_nuke(ep, 0);
	}

	dprintk(0, "a???\n");
	if(unlikely(list_empty(list)))
	{
		req = NULL;
	}
	else
	{
		req = list_entry(list->next, struct ix200_request, queue);
	}

	if(!req)
	{
		dprintk(0, "Tx: no request in queue\n");
		return ;
	}

	/* begin transfer */
	buf = req->req.buf + req->req.actual;
	size = min((uint32_t)(req->req.length - req->req.actual),
	   (ep_num?EP_MAXPACKET:EP0_MAXPACKET));
	memcpy(d_buffer, buf, size);


#if 0	
	{
		int i;
		char descx[1024];
		for (i = 0; i < size; i++)
		  sprintf(descx + 3 * i, "%02x ", d_buffer[i]);

		dprintk(2, "desc->[%s]\n", descx);
	}
#endif

	req->req.dma = dma_map_single(NULL, (void *)d_buffer, size, DMA_TO_DEVICE);

	writel(req->req.dma, __reg + USB_MDAR);

	acr = USB_ReqLength(size) |  USB_IN_Prebuffer |USB_QueryACK | USB_DMA_TxEn | USB_TxBuffer(ep_num);
	writel(acr, __reg + USB_ACR);

	for(i=0;i<10;i++);
	/* wait DMA finish */
	while(1) {
		acr = readl(__reg + USB_ACR);
		if(acr & USB_ReqError)
		{
			dprintk(1, "udc detect: dma error.\n");
			/* clear error state */
			writel(acr | USB_ReqError, __reg + USB_ACR);
			writel(readl(__reg + USB_FHHR) |
			   USB_PEP_HALT(ep_num), __reg + USB_FHHR);
			writel(readl(__reg + USB_BFCR) | 0x1, __reg + USB_BFCR);
			goto __exit_tx__;
		}
	   	else if(!(acr & 0x830))
		{
			dprintk(0, "udc detect: dma finished.\n");
			break;
		}
	}
	req->req.actual += size;

	/* transfer finished */
	if((req->req.zero && !size) ||
	   (size != ep->maxpacket) ||
	   (!req->req.zero && (req->req.actual == req->req.length)))
	{
	  __req_done(ep, req, 0);
	  dprintk(0, "esc:%d\n",size);
	}

__exit_tx__:
	dma_unmap_single(NULL, req->req.dma, size, DMA_TO_DEVICE);
	return ;
}



static void ix_handle_ep0(uint32_t isr)
{
	if(isr & USB_EP0_Setup) {
		__ep0_setup();
		writel(USB_EP0_Setup, __reg + USB_ISR);
	}
	else if(isr & USB_EP0_IN) {
		__ep_tx_post(0);
		writel(USB_EP0_IN, __reg + USB_ISR);
	}
#if 0	
	if(!ix_udc.is_address)
	{
		uint32_t dwAddressVal = readl(__reg + USB_CCR);
		dwAddressVal = dwAddressVal & 0x3f;
		if(dwAddressVal)
		{
			__address_setup(dwAddressVal);
			ix_udc.is_address = 1;
		}
	}
#endif	
	return ;
}

static void ix_handle_ep(int ep_num)
{
	uint32_t fhhr = readl(__reg + USB_FHHR);
	const struct usb_endpoint_descriptor *desc =
		ix_udc.desc[ep_num];

	dprintk(0, "%s(), ep%d\n", __func__, ep_num);
	if(desc->bEndpointAddress & USB_DIR_IN)
	{

	}
	else {                                                                                    
		/* the stall handshake from host */
		if (fhhr & USB_PEP_HALT(ep_num))
		  /* clear stall */
		  writel(fhhr | USB_PEP_HALT(ep_num), __reg + USB_FHHR);
		__ep_rx(ep_num);
	} 
	return ;
}

static void otg_cable_disconnect(struct ix200_udc *ix_udc)
{
	
	ix_udc->driver->disconnect(&ix_udc->gadget);
	ix_udc->is_config = 0;
	ix_udc->is_address = 0;
	g_rx_flag = 0;
#ifndef CONFIG_IG_OTG_AUTO
	thread_state=THREAD_DIS;
#else	
	__is_connect = 0;
	__start_check = 1;
#endif

	//start_monitor_thread();
}

static irqreturn_t ix_udc_irq(int dummy, void *dev)
{
	struct ix200_udc *udc = dev;
	uint32_t isr = readl(__reg + USB_ISR);
	uint32_t __bcisr0 = readl(__reg + USB_BCISR);
	uint32_t ier = readl(__reg + USB_IER);
	unsigned long flags;

	if(__bcisr0 & 0x8)
	{
#ifndef CONFIG_IG_OTG_AUTO 		
		if(thread_state == THREAD_EN)
			otg_cable_disconnect(udc);
#else
		if(__is_connect == 1)
			otg_cable_disconnect(udc);
#endif		

		writel(0x8, __reg + USB_BCISR);
		dprintk(1, "usb bcsr disconnect\n");
	}

	dprintk(0, "++irq:0x%x\n", isr);
	if(!isr)
	{
		dprintk(1, "no irq stat detected.\n");
		return IRQ_HANDLED;
	}

	spin_lock_irqsave(&udc->lock, flags);
	/* Driver connected ? */
	if(!udc->driver) {
		/* simply clear interrupt */
		dprintk(1, "no driver.\n");
		writel(isr, __reg + USB_ISR);
		goto __exit_irq__;
	}
	
	/* connect */
	if((isr&ier) && (isr & USB_BUS))
	{
		if(isr & USB_Connect) {
			writel(USB_Connect, __reg + USB_ISR);
			dprintk(1, "usb connect\n");
		}

		/* disconnect */
		if(isr & USB_Disconnect) {
			if(thread_state == THREAD_EN)
				otg_cable_disconnect(udc);
			writel(USB_Disconnect, __reg + USB_ISR);
			dprintk(1, "usb disconnect\n");
		}

		/* reset */
		if(isr & USB_Reset) {
			if(!ix_udc.is_config)
				writel(USB_SOF, __reg + USB_IER);
			writel(USB_Reset, __reg + USB_ISR);
			dprintk(1, "usb reset, speed=%d\n", udc->gadget.speed);
		}

		/* suspend */
		if(isr & USB_Suspend) {
			writel(USB_Suspend, __reg + USB_ISR);
			dprintk(1, "usb suspend\n");
		}

		/* resume */
		if(isr & USB_Resume) {
			writel(USB_Resume, __reg + USB_ISR);
			dprintk(1, "usb resume\n");
		}

		/* sof */
		if((isr & USB_SOF) && (ier & USB_SOF)) {
			if(!ix_udc.is_config)
			{
				uint32_t dwConfigureVal = readl(__reg + USB_CCR);
				dwConfigureVal = (dwConfigureVal & 0xf00)>>8;
				if(dwConfigureVal)
				{
					__config_setup();
					ix_udc.is_config = 1;
					writel(USB_SOF, __reg + USB_IDR);
				}
			}
			writel(USB_SOF, __reg + USB_ISR);
		}
	}
	/* ep0 */
	if(isr & USB_EP0)
	{
		ix_handle_ep0(isr);
	}

	/* eps */
	if(isr & USB_PEP)
	{
		int i;
		for(i = 1; i < IX_EP_COUNT; i++)
		  if(isr & USB_PEP_Tran(i))
		  {
			  ix_handle_ep(i);
		  }
	}

__exit_irq__:
	spin_unlock_irqrestore(&udc->lock, flags);

	return IRQ_HANDLED;
}


static void ix_udc_en(int en)
{
	if(en)
	{
		/* set phy interface */
		writel(0xffff1100, __reg + USB_PIR0);
		writel(0xffffffff, __reg + USB_PIR1);

		//ix_udc.gadget.speed = USB_SPEED_UNKNOWN;
		//ix_udc.gadget.speed = USB_SPEED_FULL;          
		ix_udc.gadget.speed = USB_SPEED_HIGH;

		dprintk(1, "reg USB_BCWR: 0x%08x\n", readl(__reg + USB_BCWR));
		dprintk(1, "reg USB_BCIER: 0x%08x\n", readl(__reg + USB_BCIER));
		dprintk(1, "reg USB_BCIDR: 0x%08x\n", readl(__reg + USB_BCIDR));
		dprintk(1, "reg USB_BCISR: 0x%08x\n", readl(__reg + USB_BCISR));
		dprintk(1, "reg USB_TBCR0: 0x%08x\n", readl(__reg + USB_TBCR0));
		dprintk(1, "reg USB_TBCR1: 0x%08x\n", readl(__reg + USB_TBCR1));
		dprintk(1, "reg USB_TBCR2: 0x%08x\n", readl(__reg + USB_TBCR2));
		dprintk(1, "reg USB_TBCR3: 0x%08x\n", readl(__reg + USB_TBCR3));
		dprintk(1, "reg USB_TBCR0: 0x%08x\n", readl(__reg + USB_TBCR0));
		dprintk(1, "reg USB_ACR: 0x%08x\n", readl(__reg + USB_ACR));
		dprintk(1, "reg USB_IER: 0x%08x\n", readl(__reg + USB_IER));
	}

	/* reset */
	writel(0x00000000, __reg + USB_BCWR);
	writel(0x00000000, __reg + USB_BCIER);
	writel(0x00000000, __reg + USB_BCIDR);
	writel(0x0000003f, __reg + USB_BCISR);

	return ;
}

static void ix_ep_init(void)
{
	uint32_t i;

	/* init gadget ep_list */
	INIT_LIST_HEAD(&ix_udc.gadget.ep_list);
	INIT_LIST_HEAD(&ix_udc.gadget.ep0->ep_list);

	for(i = 0; i < IX_EP_COUNT; i++)
	{
		ix_udc.ep[i].ops = &ix_ep_ops;
		if(i)
			list_add_tail(&ix_udc.ep[i].ep_list, &ix_udc.gadget.ep_list);

		/* Init EP queues */
		INIT_LIST_HEAD(&ix_udc.ep_req[i]);
	}
}

static struct clk *bus_clk;
static int ix_udc_probe(struct platform_device *pdev)
{
	struct ix200_udc *udc = &ix_udc;
	struct device *dev = &pdev->dev;
	struct resource *res;//, *area;
	int size, ret;

	dev_dbg(dev, "%s()\n", __func__);

	bus_clk = clk_get(NULL, "usb-bus-gadget");
	if(IS_ERR(udc->clk)) {
		dev_err(dev, "failed to get usb bus clock source\n");
		return PTR_ERR(bus_clk);
	}

	clk_enable(bus_clk);

	udc->clk = clk_get(NULL, "usb-device");
	if(IS_ERR(udc->clk)) {
		dev_err(dev, "failed to get udc clock source\n");
		return PTR_ERR(udc->clk);
	}

	clk_enable(udc->clk);

	mdelay(10);
	dprintk(DEBUG_NORMAL, "got and enabled clocks\n");

	spin_lock_init(&udc->lock);
	res = pdev->resource;
	size = res->end - res->start + 1;

	ix_udc.regbase = ioremap_nocache(res->start, size); 
	__reg = ix_udc.regbase; /* easy to use */

	if(!__reg)
	{
		dev_err(&pdev->dev, "Can not remap register address..\n");
		release_mem_region(res->start, size);
		return -EIO;
	}

	device_initialize(&udc->gadget.dev);
	udc->gadget.is_dualspeed = 1;
	udc->gadget.dev.parent = &pdev->dev;
	udc->gadget.dev.dma_mask = pdev->dev.dma_mask;

	/* allocate dma buffer */
	d_buffer = kmalloc(4096, GFP_KERNEL);
	if(!d_buffer)
	{
		dprintk(1, "allocate global buffer failed.\n");
		return -ENOMEM;
	}
	if(virt_addr_valid((uint32_t)d_buffer))
	  dprintk(1, "allocate valid global buffer %x.\n", d_buffer);
	else
	  dprintk(1, "allocated but not valid global buffer %x.\n", d_buffer);

	platform_set_drvdata(pdev, udc);

	udc->irqno = platform_get_irq(pdev, 0);
	dprintk(DEBUG_NORMAL, "udc: irq no. %d\n", udc->irqno);
	ret = request_irq(udc->irqno, ix_udc_irq, IRQF_DISABLED, pdev->name, udc);
	if(ret)
	{
		dev_err(dev, "cannot get irq %i, err %d\n", udc->irqno, ret);
		iounmap(udc->regbase);
		release_mem_region(res->start, size);
		return ret;
	}

	ix_udc.is_config = 0;
	ix_udc.is_address = 0;
	g_rx_flag = 0;
	ix_ep_init();
	dprintk(1, "probe ok\n");
	return 0;
}

static int ix_udc_remove(struct platform_device *pdev)
{
	struct ix200_udc *udc = platform_get_drvdata(pdev);
	struct resource *res;

	dev_dbg(&pdev->dev, "%s()\n", __func__);

	res = pdev->resource;

	if(udc->driver)
	  return -EBUSY;

	free_irq(udc->irqno, udc);
	iounmap(__reg);
	release_mem_region(res->start, res->end - res->start + 1);

	platform_set_drvdata(pdev, NULL);

	if(!IS_ERR(udc->clk) && udc->clk) {
		clk_disable(udc->clk);
		clk_put(udc->clk);
		udc->clk = NULL;
	}

	if(!IS_ERR(bus_clk) && bus_clk) {
		clk_disable(bus_clk);
		clk_put(bus_clk);
		bus_clk = NULL;
	}

	dev_dbg(&pdev->dev, "%s: remove ok\n", __func__);
	return 0;
}

#ifdef CONFIG_PM
static int ix_udc_suspend(
   struct platform_device *pdev, pm_message_t message)
{
	return 0;
}

static int ix_udc_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define ix_udc_suspend NULL
#define ix_udc_resume NULL
#endif

static struct platform_driver ix_udc_driver = {
	.driver		= {
		.name	= "ix-udc",
		.owner	= THIS_MODULE,
	},
	.probe		= ix_udc_probe,
	.remove		= ix_udc_remove,
	.suspend	= ix_udc_suspend,
	.resume		= ix_udc_resume,
};

static int __init ix_udc_init(void)
{
	printk(KERN_INFO "iMAPx200 USB Device controller (c) 2009~2014\n");
	return 
		platform_driver_register(&ix_udc_driver);
}

static void __exit ix_udc_exit(void)
{
	platform_driver_unregister(&ix_udc_driver);
}

module_init(ix_udc_init);
module_exit(ix_udc_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("warits <warits.wang@infotmic.com.cn>");
MODULE_DESCRIPTION("UDC driver for iMAPx200");
MODULE_VERSION("v0");
