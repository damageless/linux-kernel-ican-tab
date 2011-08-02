/***************************************************************************** 
 ** imapx200_cam.c 
 ** 
 ** Copyright (c) 2009~2014 ShangHai Infotm Ltd all rights reserved. 
 ** 
 ** This program is free software; you can redistribute it and/or modify
 ** it under the terms of the GNU General Public License as published by
 ** the Free Software Foundation; either version 2 of the License, or
 ** (at your option) any later version.
 ** 
 ** Description: main file of imapx200 media encode driver
 **
 ** Author:
 **     neville <haixu_fu@infotm.com>
 **      
 ** Revision History: 
 ** ­­­­­­­­­­­­­­­­­ 
 ** 1.1  06/24/2010 neville 
 ** 2.0  11/08/2010 neville
 *******************************************************************************/
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#include <plat/clock.h>



#include <mach/imapx_base_reg.h>
#include <mach/irqs.h>
#include <mach/imap_hwid.h>
#include "imapx200_cam.h"

static struct class *camif_class;
unsigned int frame = 0;
struct clk  *imap_cam_clk;

#define FLAG_SIZE    	4096
unsigned int    flag_phy_addr = 0;
unsigned int *  flag_vir_addr = NULL;

static unsigned int camif_open_count;

static wait_queue_head_t wait_camif;
static volatile int pr_flag;
static volatile int co_flag;
static struct mutex pr_mutex;
static struct mutex co_mutex;
static struct mutex flag_mutex;
static struct mutex normal_mutex;
struct imapx200_camif_param_t  *param;
struct timeval time1, time2;

LIST_HEAD(sensor_pool);

static int imapx200_cam_open(struct inode *inode, struct file *file)
{
	printk(KERN_ERR "imapx200_cam_open()\n");
	param->ops->power_on();
	camif_open_count++;             
	return CAMIF_RET_OK;
}       


static int imapx200_cam_release(struct inode *inode, struct file *file)
{
	printk(KERN_ERR "imapx200_cam_release()\n");
	param->ops->power_off();
	camif_open_count--;
	return CAMIF_RET_OK;
}

static int imapx200_cam_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg) 
{
	int value;
	int ret = -1;
	switch(cmd)
	{
		case PR_WAIT_OK:
			mutex_lock(&pr_mutex);
			pr_flag = 1;
			mutex_unlock(&pr_mutex);
			wait_event_interruptible_timeout(wait_camif, pr_flag == 0, HZ/4);
			break;
		case CO_WAIT_OK:
			mutex_lock(&co_mutex);
			co_flag = 1;
			mutex_unlock(&co_mutex);
			wait_event_interruptible_timeout(wait_camif, co_flag == 0, HZ/ 4);
			break;
		case SENSOR_SET_MODE:
			if(copy_from_user(&value, (int *)arg, sizeof(int)))
			{
				ret = -EFAULT;
			}

			if(param->ops->set_mode)
			{
				if(param->ops->set_mode(value,param) == 0)
					ret = CAMIF_RET_OK;
				else{
					ret =  -1;
				}
			} else {
				ret = -1;
			}
			break;

		case SENSOR_SET_WHITE_BALANCE:
			if(copy_from_user(&value, (int *)arg, sizeof(int)))
			{
				ret = -EFAULT;
				return ret;
			}
			if(param->ops->set_wb)
			{
				if(param->ops->set_wb(value) == 0)
					ret = CAMIF_RET_OK;
				else
					ret = -1;
			}
			else 
				ret = -1;
			break;
		case SENSOR_SET_COLOR_EFECT:
			if(copy_from_user(&value, (int *)arg, sizeof(int)))
			{
				ret = -EFAULT;
				return ret;
			}
			if(param->ops->set_effect)
			{
				if(param->ops->set_effect(value) == 0)
					ret = CAMIF_RET_OK;
				else 
					ret = -1;
			}
			else 
				ret = -1;
			
			break;
		case SENSOR_SET_ANTIBANDING:
			ret = -1;
			break;
		case SENSOR_SET_BRIGHTNESS:
			ret = -1;
			break;
		case SENSOR_SET_NIGHT_MODE:
			ret = -1;
			/*
			if(copy_from_user(&value, (int *)arg, sizeof(int)))
			{
				ret = -EFAULT;
				return ret;
			}
			
			switch(value)
			{
				case SENSOR_SCENCE_AUTO:
					imapx200_cam_night_mode_off();
					break;
				case SENSOR_SCENCE_NIGHT:
					imapx200_cam_night_mode_on();
					break;
				default:
					break;
			}
			*/
			break;
		case GET_FLAG_PHY:                                              
			if(copy_to_user((unsigned int *)arg, &flag_phy_addr, 4))
			{                                                       
				printk("copy_to_usr error\n");                  
				return -EFAULT;  
			}                                      
			ret = CAMIF_RET_OK;
			break;                                                    
		case SET_FLAG_DIRTY:                                            
			mutex_lock(&flag_mutex);                                  
			flag_vir_addr[0] = 0xFFFF;                                
			mutex_unlock(&flag_mutex);                                
			ret = CAMIF_RET_OK;
			break;                                                    

		default:
			ret = -1;
			break;

	}

	return ret;
}

static struct file_operations imapx200_cam_fops = 
{
		owner:		THIS_MODULE,
		open:		imapx200_cam_open,
		release:	imapx200_cam_release,
		ioctl:		imapx200_cam_ioctl,
};


static irqreturn_t imapx200_cam_irq_handle(int irq ,void *dev_id)
{
	u32 intmask;
	unsigned int clear_int = 0;
	irqreturn_t ret;
	intmask = readl(param->ioaddr+IMAP_CICPTSTATUS);

	if(!intmask || intmask == 0xFFFFFFFF)
	{
		ret = IRQ_NONE;
		return ret;
	}
	if(intmask & CAMIF_OVERFLOW)
	{
		camif_error("get IRQ %08x\n",intmask);
		clear_int |= (intmask & CAMIF_OVERFLOW);
	}
	if(intmask & CAMIF_UNDERFLOW)
	{
		camif_error("get IRQ %08x\n",intmask);
		clear_int |= (intmask & CAMIF_UNDERFLOW);
	}

	if(intmask & PRFIFO_DIRTY)
	{
		camif_error("get IRQ %08x\n",intmask);
		clear_int |= PRFIFO_DIRTY;
	}
	if(intmask & COFIFO_DIRTY)
	{
		camif_error("get IRQ %08x\n",intmask);
		clear_int |= COFIFO_DIRTY;
	}
	if(intmask & CERR656)
	{
		camif_error("get IRQ %08x\n",intmask);
		clear_int |= CERR656;
	}

	if(intmask & PR_LEISURE)
	{
		camif_error("get IRQ %08x\n",intmask);
		clear_int |= PR_LEISURE;
	}

	if(intmask & CO_LEISURE)
	{
		camif_error("get IRQ %08x\n",intmask);
		clear_int |= CO_LEISURE;
	}

	if(intmask & PR_DMA_SUCCESS)
	{
	//	camif_error("get pr IRQ %08x\n",intmask);
		mutex_lock(&pr_mutex);

		if(pr_flag == 1)
		{
			wake_up_interruptible(&wait_camif);
		}
		pr_flag = 0;
		mutex_unlock(&pr_mutex);
		clear_int |= PR_DMA_SUCCESS;
	}

	if(intmask & CO_DMA_SUCCESS)
	{
		camif_error("get pr IRQ %08x\n",intmask);
		mutex_lock(&co_mutex);
		if(co_flag == 1)
		{
			wake_up_interruptible(&wait_camif);
		}
		co_flag = 0;
		mutex_unlock(&co_mutex);
		clear_int |= CO_DMA_SUCCESS;
	}

	writel(intmask, param->ioaddr+IMAP_CICPTSTATUS);

	return IRQ_HANDLED;
}

static int imapx200_cam_decide_sensor(struct imapx200_camif_param_t *param)
{
	struct sensor_ops *p;

	list_for_each_entry(p, &sensor_pool, link)
	{
		param->ops = p;

#if defined(CONFIG_IG_CAM_CMOS_ID)
		if(!p || !p->idlist) continue;

		p->reset(param);
		p->power_on();
		mdelay(10);

		{ uint32_t *q, id = p->get_id();
			for(q = p->idlist; *q; q++)
			  if(*q == id) {p->power_off();
				  goto __cam_detected__;}
		}

		p->power_off();
#elif defined(CONFIG_IG_CAM_HWID)
		if(p && (p->hwid ==
			   imap_product_getid(IMAP_PRODUCT_CAMERA)))
		  goto __cam_detected__;
#else
		/* use the first sensor in list */
		goto __cam_detected__;
#endif
	}

	printk(KERN_INFO "no camif sensor is available.\n");
	return -1;

__cam_detected__:
	if(p->name)
	  printk(KERN_INFO "sensor %s is detected.\n", p->name);

	return 0;
}

static int imapx200_cam_do_en(int en)
{
	/*
	 * camera interface power supply enable
	 *
	 */
	uint32_t pow_addr, pwdn_addr;

	pow_addr = __imapx_name_to_gpio(CONFIG_IG_CAMIF0_SUPPLY);
	if(pow_addr == IMAPX_GPIO_ERROR) {                       
		printk(KERN_ERR "failed to get pow_supply pin.\n");   
	} else {                                                        
		imapx_gpio_setcfg(pow_addr, IG_OUTPUT, IG_NMSL);               
		imapx_gpio_setpin(pow_addr, !!en, IG_NMSL);               
		imapx_gpio_chmod(pow_addr, (en?IG_NORMAL: IG_SLEEP));
	}

	/*
	 *    camera inerface pwdn enable
	 *
	 */
	pwdn_addr = __imapx_name_to_gpio(CONFIG_IG_CAMIF0_PND);
	if(pow_addr == IMAPX_GPIO_ERROR) {                       
		printk(KERN_ERR "failed to get pwdn pin.\n");   
	} else {
		imapx_gpio_setcfg(pwdn_addr, IG_OUTPUT, IG_NMSL);               
		imapx_gpio_setpin(pwdn_addr, (!!en) ^ (!!param->ops->pwdn), IG_NMSL);
		imapx_gpio_chmod(pow_addr, (en?IG_NORMAL: IG_SLEEP));
	}

	printk(KERN_ERR "camif en=%d\n", en);

	return 0;
}

static inline int imapx200_cam_default_on(void) {
	return imapx200_cam_do_en(1);
}
static inline int imapx200_cam_default_off(void) {
	return imapx200_cam_do_en(0);
}

static int imapx200_cam_i2c_read(uint8_t *buf,
   uint8_t *addr, uint32_t size, uint32_t len)
{
	struct i2c_msg msgs[] = {{.addr = param->ops->addr, .flags = I2C_M_NOSTART,
		.len = size, .buf = addr}, {.addr = param->ops->addr,
			.flags = I2C_M_RD, .len = len, .buf = buf}};

	if(param->ops->adapter)
	  if(2 == i2c_transfer(param->ops->adapter, msgs, 2))
		return 0;
	return -1;
}

static int imapx200_cam_i2c_write(uint8_t *buf, uint32_t len)
{
	struct i2c_msg msgs[] = {{.addr = param->ops->addr, .flags= 0,
		.len = len, .buf = buf} };

	if(param->ops->adapter)
	  if(1 == i2c_transfer(param->ops->adapter, msgs, 1))
		return 0;
	return -1;
}

static int imapx200_cam_fill_pointer(struct sensor_ops *ops)
{
	/* fill the pointer is not specified. */
	if(!ops->power_on)
	  ops->power_on = imapx200_cam_default_on;
	if(!ops->power_off)
	  ops->power_off = imapx200_cam_default_off;
	if(!ops->i2c_write)
	  ops->i2c_write = imapx200_cam_i2c_write;
	if(!ops->i2c_read)
	  ops->i2c_read = imapx200_cam_i2c_read;

	if(!(ops->adapter = i2c_get_adapter(CONFIG_IG_CAMIF0_I2C + 1)))
	{
		printk(KERN_ERR "can not get i2c adapter for camif0.\n");
		return -1;
	}

	return 0;
}

int imapx200_cam_sensor_register(struct sensor_ops *ops)     
{                                                                
	if(ops) {
		imapx200_cam_fill_pointer(ops);
		list_add_tail(&ops->link, &sensor_pool);
		if(ops->name)
		  printk(KERN_INFO "adding camif sensor: %s\n", ops->name);
	} else {
		printk(KERN_ERR "try to register NULL camif ops\n");
		return -1;
	}

	return 0;
}                                                                

int imapx200_cam_sensor_unregister(struct sensor_ops *ops)
{                       
	if(ops) {
		list_del_init(&ops->link);
		if(ops->name)
		  printk(KERN_INFO "camif sensor: %s, unregistered\n", ops->name);
	} else {
		printk(KERN_ERR "try to unregister NULL camif ops\n");
		return -1;
	}

	return 0;
}       

static int imapx200_cam_gpio_init(void)
{
	u32 tmp;

	/*
	 * config clock for camera interface
	 */
	tmp = readl(rDIV_CFG1);
	tmp &= ~((3 << 16) | (0x1f << 18));
	tmp |= ((2<<16) | (19<<18));
	writel(tmp, rDIV_CFG1);

	/*
	 * config GPL IO for camera interface
	 */
	imapx_gpio_setcfg(IMAPX_GPL_ALL, IG_CTRL0, IG_NORMAL);

	/* FIXME: do not forget to power off */
	return 0;
}

static int imapx200_cam_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;

	imap_cam_clk = NULL;
	camif_open_count = 0;


	init_waitqueue_head(&wait_camif);
	pr_flag = co_flag = 0;
	camif_debug("iMAP camera driver probe\n");

	param = kzalloc(sizeof(struct imapx200_camif_param_t), GFP_KERNEL);
	if(!param) {
		camif_error("alloc buffer failed!\n");
		return  -ENOMEM;
	}

	param->dev = &pdev->dev;

	param->hclk = clk_get(&pdev->dev, "camif");
	if( IS_ERR (param->hclk)) {
		camif_error("failed to get clock\n");
		ret = -ENOENT;
		goto err_io_noclk;
	}
	clk_enable(param->hclk);

	param->irq = platform_get_irq(pdev,0);
	if(param->irq < 0) {
		camif_error("no irq specified\n");
		ret = param->irq;
		goto err_io_clk;
	}

	ret = request_irq(param->irq, imapx200_cam_irq_handle, IRQF_DISABLED,
			dev_name(&pdev->dev),param);
	if(ret)
		goto err_io_clk;



	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res) {
		camif_error("no memory specified\n");
		ret = -ENOENT;
		goto err_io_clk;
	}

	param->phy_start = res->start;
	param->phy_size = resource_size(res);
	param->res = request_mem_region(res->start, resource_size(res),
			pdev->name);

	if(!param->res) {
		camif_error("cannot request IO\n");
		ret = -ENXIO;
		goto err_io_clk;
	}


	param->ioaddr = ioremap_nocache(res->start, resource_size(res));
	if(!param->ioaddr) {
		camif_error("cannot map IO\n");
		ret = -ENXIO;
		goto err_io_mem;
	}

	imapx200_cam_gpio_init();
	printk(KERN_INFO "decide-sensor\n");
	if(imapx200_cam_decide_sensor(param) != 0)
	{
		printk(KERN_INFO "Unsuport sensor\n");
		ret = -ENXIO;
		goto err_io_mem;
	}

	
	/* reset controller and power off */
	param->ops->reset(param);
	param->ops->power_off();

	memset(&pr_mutex, 0x00, sizeof(struct mutex));
	memset(&co_mutex, 0x00, sizeof(struct mutex));
	memset(&flag_mutex, 0x00, sizeof(struct mutex));
	memset(&normal_mutex, 0x00, sizeof(struct mutex));
	mutex_init(&normal_mutex);
	mutex_init(&pr_mutex);
	mutex_init(&co_mutex);
	mutex_init(&flag_mutex);

	flag_vir_addr = (unsigned int  *)kmalloc(FLAG_SIZE, GFP_KERNEL);
	if(flag_vir_addr == NULL)                      
	{                                              
		camif_error("kmalloc buffer failed\n");
		ret = -ENOMEM;                       
		goto err_io_mem;                       
	}                                              
	memset(flag_vir_addr, 0x00, FLAG_SIZE);
	flag_phy_addr = (unsigned int)virt_to_phys((unsigned long *)flag_vir_addr);
	camif_debug("[camif]-:flag_phy_addr = %0x\n",flag_phy_addr);                    

	ret = register_chrdev(CAMIF_DEFAULT_MAJOR, "imapx200_camif", &imapx200_cam_fops);
	if(ret)
	{
		camif_error("imapx register chardev error!\n");
		goto out;
	}

	camif_class = class_create(THIS_MODULE, "imapx200_camif");
	param->dev_id = MKDEV(CAMIF_DEFAULT_MAJOR, CAMIF_DEFAULT_MINOR);
	device_create(camif_class , NULL,  param->dev_id, NULL, "imapx200-camif");


	return CAMIF_RET_OK;		
err_io_mem:
	release_resource(param->res);
err_io_clk:
	clk_disable(param->hclk);
err_io_noclk:
	kfree(param);
out:
	camif_error("driver probe failed with err %d\n",ret);
	return ret;
}


static int imapx200_cam_remove(struct platform_device *pdev) 
{
	mutex_destroy(&pr_mutex);
	mutex_destroy(&co_mutex);
	mutex_destroy(&flag_mutex);
	mutex_destroy(&normal_mutex);

	iounmap((void *)(param->ioaddr));		
	release_mem_region(param->phy_start, param->phy_size);
	if(param->res)
	{
		release_resource(param->res);
		kfree(param->res);
		param->res = NULL;
	}

	free_irq(param->irq, pdev);
	device_destroy(camif_class, param->dev_id);
	class_destroy(camif_class);
	unregister_chrdev(CAMIF_DEFAULT_MAJOR, "imapx200-camif");
	clk_disable(param->hclk);


	return CAMIF_RET_OK;
}

#ifdef CONFIG_PM
static int imapx200_cam_suspend(struct platform_device *pdev, pm_message_t state)
{
	param->ops->power_off();
	return CAMIF_RET_OK;
}

static int imapx200_cam_resume(struct platform_device *pdev)
{
	param->ops->reset(param);
	if(camif_open_count)
	  param->ops->power_on();
	return CAMIF_RET_OK;
}
#else 

#define	imapx200_cam_suspend NULL
#define	imapx200_cam_resume  NULL
#endif


static struct platform_driver imapx200_cam_driver =
{
	.probe			= imapx200_cam_probe,
	.remove			= imapx200_cam_remove,
#ifdef CONFIG_PM
	.suspend		= imapx200_cam_suspend,
	.resume			= imapx200_cam_resume,
#endif	
	.driver			=
	{
		.owner		= THIS_MODULE,
		.name 		= "imapx200_camif",
	},
};


static int __init imapx200_cam_init(void)
{
	printk(KERN_INFO "imapx200_cam_init ++\n");

	if(platform_driver_register(&imapx200_cam_driver))
	{
		camif_error("Failed to register IMAPX200 CAMIF driver\n");
		return -EPERM;
	}
	camif_debug("IMAPX200 CAMIF driver register OK!\n");

	return CAMIF_RET_OK;
}


static void __exit imapx200_cam_exit(void)
{
	platform_driver_unregister(&imapx200_cam_driver);
	camif_debug("IMAPX200 CAMIF driver unregister OK!\n");
}



module_init(imapx200_cam_init);
module_exit(imapx200_cam_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("neville of infoTM");
MODULE_DESCRIPTION("IMAPX200 CAMIF DRIVER");

