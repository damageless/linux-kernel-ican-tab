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
#include <linux/ioctl.h>
#include <linux/clk.h>
#include <asm/delay.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <mach/imapx_gpio.h>
#include <linux/semaphore.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
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
struct sensor_orientation {
	char buff[3];
	int buffersize;
};

enum sensor_state {
	sensor_init,
	sensor_top
};
struct sensor_orientation *sensor_dev; 

static unsigned int Sensor_IIC_Write(unsigned char IICAddr, unsigned char ByteAddr, unsigned char
   Data, unsigned int Size);                                                                        
static unsigned int Sensor_IIC_Read(unsigned char IICAddr, unsigned char ByteAddr, unsigned char* Data, unsigned int Size)
{
	struct i2c_adapter *adapter;
	unsigned char *buf = kmalloc(Size, GFP_KERNEL);

	struct i2c_msg msgs1[] = { 
		{
			.addr   = IICAddr,
			.flags  = 0,
			.len            = 1,
			.buf            =&ByteAddr,
		},{
			.addr   = IICAddr,
			.flags  = I2C_M_RD,
			.len            = Size,
			.buf            = Data,
		}
	};
	if (!buf)
	{
		printk(KERN_ERR "[IIC_Read]: unable to allocate memory for Read.\n");
		return -1; 
	}

	adapter = i2c_get_adapter(CONFIG_ACC_FS7600_I2C + 1);
	if (!adapter)
	{
		printk(KERN_ERR "[IIC_Read]: can't get i2c adapter\n");

		return -1; 
	}

	if (i2c_transfer(adapter, msgs1, 2) != 2)
	{
		//		printk("In function %s line %d data read is %x\n",__func__,__LINE__,*Data);
		return -1; 
	}
	//	printk("In function %s line %d data read is %x\n",__func__,__LINE__,*Data);

	kfree(buf);

	return 0;
}


/*
 * open system call just mark file private data as a sensor 
 * instance by default, and you can change it by a ioctl call
 */
static int imapx200_sensor_open(struct inode *inode, struct file *file)
{  

	file->private_data = &(sensor_param.dec_instance);

	sensor_debug("IMAPX200 Decode open OK\n");

	return IMAPX200_SENSOR_RET_OK;
}
static int imapx200_sensor_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int i, index= -1, ret = -1;
	char del_buff[]={1,1,1};
	char indicator[] = CONFIG_ACC_FS7600_DIRECTION;
	char directions[] = {'d', 0, 1, 1, 'r', 0, 1, 0, 'l', 1, 1, 0, 'u', 1, 0, 0};

	/* dec instance by default, you can change it by ioctl pp instance */
	sensor_debug("IMAPX200 read OK\n");

	Sensor_IIC_Read(SENSOR_ADDR,SENSOR_XOUT,&sensor_dev->buff[0],1);
	sensor_dev->buff[0]=(sensor_dev->buff[0]&0x3f);
	Sensor_IIC_Read(SENSOR_ADDR,SENSOR_YOUT,&sensor_dev->buff[1],1);
	sensor_dev->buff[1]=(sensor_dev->buff[1]&0x3f);
	Sensor_IIC_Read(SENSOR_ADDR,SENSOR_ZOUT,&sensor_dev->buff[2],1);
	sensor_dev->buff[2]=(sensor_dev->buff[2]&0x3f);

    //    printk(KERN_EMERG"x is %d,y is %d,z is %d",sensor_dev->buff[0],sensor_dev->buff[1],sensor_dev->buff[2]);
	if (sensor_dev->buff[0]<=63 && sensor_dev->buff[1]<=63
	   && sensor_dev->buff[2]<=63)
	{
		if (((sensor_dev->buff[0]>0 && sensor_dev->buff[0]<25)
			   || (sensor_dev->buff[0]>45 && sensor_dev->buff[0]<63))
		   && (sensor_dev->buff[1]>40 && sensor_dev->buff[1]<45))
		  index = 0;
		if (((sensor_dev->buff[1]>45 && sensor_dev->buff[1]<63)
			   || (sensor_dev->buff[1]>0 && sensor_dev->buff[1]<25))
		   && (sensor_dev->buff[0]>40 && sensor_dev->buff[0]<50))
		  index = 1;
		if (((sensor_dev->buff[1]>45 && sensor_dev->buff[1]<63)
			   || (sensor_dev->buff[1]>0 && sensor_dev->buff[1]<25))
		   && (sensor_dev->buff[0]>15 && sensor_dev->buff[0]<25))
		  index = 2;
		if (((sensor_dev->buff[0]>0 && sensor_dev->buff[0]<15)
			   ||(sensor_dev->buff[0]>45 && sensor_dev->buff[0]<63))
		   && (sensor_dev->buff[1]>10 && sensor_dev->buff[1]<25))
		  index = 3;

	//	printk(KERN_INFO"*********direction is %d ********\n",ARRAY_SIZE(directions));
//		printk(KERN_INFO"********* index  is %d ********\n",index);
		if(index == -1)
		{
			ret = copy_to_user(buf,del_buff,3);
			if(ret)
				printk(KERN_ERR "send direction failed.\n");
			return count;
		}      
		for(i = 0; i < ARRAY_SIZE(directions) - 3; i++)
		  if(directions[i] == indicator[index]) {
			  ret = copy_to_user(buf, directions + i + 1, 3);
			  break;
		  }

		if(ret)
		  printk(KERN_ERR "send direction failed.\n");

		return count;
	}

	return 0;
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


static unsigned int Sensor_IIC_Write(unsigned char IICAddr, unsigned char ByteAddr, unsigned char Data, unsigned int Size)
{
	struct i2c_adapter *adapter;
	unsigned char *buf = kmalloc(Size + 1, GFP_KERNEL);
	struct i2c_msg msgs[] = { 
		{
			.addr   = IICAddr,
			.flags  = 0,
			.len            = Size + 1,
			.buf            = buf,
		}
	};

	if (!buf)
	{
		printk(KERN_ERR "[IIC_Write]: unable to allocate memory for EDID.\n");
		return -1; 
	}
	buf[0] = ByteAddr;
	buf[1] = Data;


	adapter = i2c_get_adapter(CONFIG_ACC_FS7600_I2C + 1);
	if (!adapter)
	{
		printk(KERN_ERR "[IIC_Write]: can't get i2c adapter\n");

		return -1; 
	}

	if (i2c_transfer(adapter, msgs, 1) != 1)
	  return -1; 

	kfree(buf);

	return 0;
}

/*
 * platform operation relate functions
 */
static int imapx200_sensor_probe(struct platform_device *pdev)
{
	int ret;

	Sensor_IIC_Write(SENSOR_ADDR,SENSOR_MODE,0x78,1);
	Sensor_IIC_Write(SENSOR_ADDR,SENSOR_SR,0x18,1);
	//  Sensor_IIC_Write(SENSOR_ADDR,SENSOR_SPCNT,0xaf,1);
	Sensor_IIC_Write(SENSOR_ADDR,SENSOR_PDET,0xe0,1);

	Sensor_IIC_Write(SENSOR_ADDR,SENSOR_MODE,0x79,1);

	ret = -1;

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
	//	sensor_driver_unregister();
	return IMAPX200_SENSOR_RET_OK;
}

#ifdef CONFIG_PM
static int imapx200_sensor_suspend(struct platform_device *pdev, pm_message_t state)
{
	//  Sensor_IIC_Write(SENSOR_ADDR,SENSOR_MODE,0x78,1);
	sensor_debug("IMAPX200_SENSOR_SUSPEND\n");
	return IMAPX200_SENSOR_RET_OK;
}

static int imapx200_sensor_resume(struct platform_device *pdev)
{
	Sensor_IIC_Write(SENSOR_ADDR,SENSOR_MODE,0x78,1);
	Sensor_IIC_Write(SENSOR_ADDR,SENSOR_SR,0x18,1);
	//  Sensor_IIC_Write(SENSOR_ADDR,SENSOR_SPCNT,0xaf,1);
	Sensor_IIC_Write(SENSOR_ADDR,SENSOR_PDET,0xe0,1);

	Sensor_IIC_Write(SENSOR_ADDR,SENSOR_MODE,0x79,1);
	sensor_debug("IMAPX200_SENSOR_RESUME\n");

	return IMAPX200_SENSOR_RET_OK;
}
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
	//enum sensor_state sensor_state_ori;
	if(platform_driver_register(&imapx200_sensor_driver))
	{
		sensor_error("Fail to register platform driver for IMAPX200 Decode Driver\n");
		return -EPERM;
	}


	sensor_dev = kmalloc(sizeof(struct sensor_orientation), GFP_KERNEL);
	if (sensor_dev == NULL) {
		sensor_debug("malloc sensor_dev failed\n");
		return -1;
	}
	memset(sensor_dev, 0, sizeof(struct sensor_orientation));
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


