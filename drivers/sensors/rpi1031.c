/***************************************************************************** 
 * sensor.c 
 * 
 * Copyright (c) 2009~2014 ShangHai Infotm Ltd all rights reserved. 
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * Description: main file of imapx200 media sensor driver
 *
 * Author:
 *     Sololz <sololz@infotm.com>
 *      
 * Revision History: 
 * ­­­­­­­­­­­­­­­­­ 
 * 1.1  12/9/2009 Sololz 
 ******************************************************************************/

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
#include <linux/gpio.h>
#include <linux/ioctl.h>
#include <linux/clk.h>
#include <asm/delay.h>
#include <mach/imapx_gpio.h>
#include <linux/semaphore.h>
#include <linux/workqueue.h>
/*
 * XXX: sensor hard ware directly connect to system bus, there is 
 * no use to clock set
 */ 
#include <plat/clock.h>

#include <mach/imapx_base_reg.h>
#include <mach/irqs.h>

#include "common_sensor.h"

/*
 * functions delare
 */
static int reset_hw_reg_sensor(void);		/* this function just set all resigster to be 0 */
static int sensor_driver_register(void);	/* register driver as an char device */
static int sensor_driver_unregister(void);

/*
 * this structure include global varaibles
 */
sensor_param_t	sensor_param;		/* global variables group */

unsigned int rpi_bit0, rpi_bit1;

/*
 * open system call just mark file private data as a sensor 
 * instance by default, and you can change it by a ioctl call
 */
static int imapx200_sensor_open(struct inode *inode, struct file *file)
{
	/* dec instance by default, you can change it by ioctl pp instance */
	file->private_data = &(sensor_param.dec_instance);

	sensor_debug("IMAPX200 Decode open OK\n");

	return IMAPX200_SENSOR_RET_OK;
}
static int imapx200_sensor_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int i, index, ret = -1;
	char indicator[] = CONFIG_ACC_RPI1031_DIRECTION;
	char directions[] = {'d', 0, 1, 1, 'r', 0, 1, 0, 'l', 1, 1, 0, 'u', 1, 0, 0};

	/* dec instance by default, you can change it by ioctl pp instance */
	sensor_debug("IMAPX200 read OK\n");

	index = ((imapx_gpio_getpin(rpi_bit1, IG_NORMAL) & 0x1) << 1) |
		(imapx_gpio_getpin(rpi_bit0, IG_NORMAL) & 0x1);

	for(i = 0; i < ARRAY_SIZE(directions) - 3; i++)
	  if(directions[i] == indicator[index]) {
		  ret = copy_to_user(buf, directions + i + 1, 3);
		  break;
	  }

	if(ret)
	  printk(KERN_ERR "send direction failed.\n");

//	printk(KERN_ERR "%c.%d\n", indicator[index], count);
	return count;
}


/*
 * fasync system call be called here
 */
static int imapx200_sensor_release(struct inode *inode, struct file *file)
{
	sensor_debug("IMAPX200 Decode release OK\n");
	return IMAPX200_SENSOR_RET_OK;
}

static struct file_operations imapx200_sensor_fops = 
{
	owner:		THIS_MODULE,
	open:		imapx200_sensor_open,
	read:		imapx200_sensor_read,
	release:	imapx200_sensor_release,
};

/*
 * platform operation relate functions
 */
static int imapx200_sensor_probe(struct platform_device *pdev)
{
	int ret = 0;

	rpi_bit0 = __imapx_name_to_gpio(CONFIG_ACC_RPI1031_BIT0);
	rpi_bit1 = __imapx_name_to_gpio(CONFIG_ACC_RPI1031_BIT1);

	if((rpi_bit0 == IMAPX_GPIO_ERROR) ||
	   (rpi_bit1 == IMAPX_GPIO_ERROR))
	{
		printk(KERN_ERR "can not get rpi1031 pin. g-sensor init failed.\n");
		return -1;
	}

	imapx_gpio_setcfg(rpi_bit0, IG_INPUT, IG_NORMAL);
	imapx_gpio_setcfg(rpi_bit1, IG_INPUT, IG_NORMAL);

	ret = sensor_driver_register();
	if(ret)
	{
		sensor_error("Fail to register char device for IMAPX200 Decode\n");
		return ret;
	}

	sensor_debug("IMAPX200 Decode Driver probe OK\n");
	return IMAPX200_SENSOR_RET_OK;
}

static int imapx200_sensor_remove(struct platform_device *pdev)
{
	/* release irq */
	sensor_debug("IMAPX200_SENSOR_REMOVE\n");

	sensor_driver_unregister();
	return IMAPX200_SENSOR_RET_OK;
}

#ifdef CONFIG_PM
static int
imapx200_sensor_suspend(struct platform_device *pdev, pm_message_t state) {
	return 0; }
static int
imapx200_sensor_resume(struct platform_device *pdev) {
	return 0; }
#endif

static struct platform_driver imapx200_sensor_driver = 
{
	.probe		= imapx200_sensor_probe,
	.remove		= imapx200_sensor_remove,
#ifdef CONFIG_PM
	.suspend	= imapx200_sensor_suspend,
	.resume		= imapx200_sensor_resume,
#endif
	.driver		=
	{
		.owner		= THIS_MODULE,
		.name		= "imapx200-sensor-orientation",
	},
};

/*
 * init and exit
 */
static int __init imapx200_sensor_init(void)
{
	/* call probe */
	if(platform_driver_register(&imapx200_sensor_driver))
	{
		sensor_error("Fail to register platform driver for IMAPX200 Decode Driver\n");
		return -EPERM;
	}

	return IMAPX200_SENSOR_RET_OK;
}

static void __exit imapx200_sensor_exit(void)
{
	/* call remove */
	platform_driver_unregister(&imapx200_sensor_driver);

	sensor_debug("IMAPX200 Decode Driver exit OK\n");
}

module_init(imapx200_sensor_init);
module_exit(imapx200_sensor_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sololz of InfoTM");
MODULE_DESCRIPTION("IMAPX200 Decode Driver");

/*
 * just write 0 to all registers to reset harware
 * TODO: we have check whether it's needed
 */
int reset_hw_reg_sensor(void)
{
	return IMAPX200_SENSOR_RET_OK;
}

/*
 * this function do driver register to regist a node under /dev
 */
static struct class *sensor_class;

int sensor_driver_register(void)
{
	int ret;

	ret = -1;
	ret = register_chrdev(SENSOR_DEFAULT_MAJOR, "imapx200-sensor", &imapx200_sensor_fops);
	if(ret < 0)
	{
		sensor_error("register char deivce error\n");
		return IMAPX200_SENSOR_RET_ERROR;
	}

	sensor_class = class_create(THIS_MODULE, "imapx200-sensor");
	device_create(sensor_class, NULL, MKDEV(SENSOR_DEFAULT_MAJOR, SENSOR_DEFAULT_MINOR), NULL, "imapx200-sensor");

	return IMAPX200_SENSOR_RET_OK;
}

int sensor_driver_unregister(void)
{
	device_destroy(sensor_class, MKDEV(SENSOR_DEFAULT_MAJOR, SENSOR_DEFAULT_MINOR));
	class_destroy(sensor_class);
	unregister_chrdev(SENSOR_DEFAULT_MAJOR, "imapx200-sensor");

	return IMAPX200_SENSOR_RET_OK;
}


