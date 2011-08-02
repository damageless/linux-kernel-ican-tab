/*  $Date: 2009/05/22 11:51:35 $
 *  $Revision: 1.1 $ 
 */

/*
* Copyright (C) 2009 Bosch Sensortec GmbH
*
*	BMA150 linux driver
* 
* Usage:	BMA150 driver by i2c for linux
*
* 
* Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in 
  compliance with the License and the following stipulations. The Apache License , Version 2.0 is applicable unless 
  otherwise stated by the stipulations of the disclaimer below. 
 
* You may obtain a copy of the License at 

    http://www.apache.org/licenses/LICENSE-2.0
  
 

Disclaimer 

 * Common:
 * This Work is developed for the consumer goods industry. It may only be used 
 * within the parameters of the respective valid product data sheet.  The Work 
 * provided with the express understanding that there is no warranty of fitness for a particular purpose. 
 * It is not fit for use in life-sustaining, safety or security sensitive systems or any system or device 
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition, 
 * the Work is not fit for use in products which interact with motor vehicle systems.  
 * The resale and/or use of the Work are at the purchaser’s own risk and his own responsibility. The 
 * examination of fitness for the intended use is the sole responsibility of the Purchaser. 
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for 
 * incidental, or consequential damages, arising from any Work or Derivative Work use not covered by the parameters of 
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch 
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased Work and Derivative Works, particularly with regard to 
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid 
 * technical specifications of the product series. They are therefore not intended or fit for resale to third 
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an 
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec 
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the 
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering 
 * samples.
 *
 * Special:
 * This Work and any related information (hereinafter called "Information") is provided free of charge 
 * for the sole purpose to support your application work. The Woek and Information is subject to the 
 * following terms and conditions: 
 *
 * The Work is specifically designed for the exclusive use for Bosch Sensortec products by 
 * personnel who have special experience and training. Do not use this Work or Derivative Works if you do not have the 
 * proper experience or training. Do not use this Work or Derivative Works fot other products than Bosch Sensortec products.  
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no 
 * responsibility for the consequences of use of such Information nor for any infringement of patents or 
 * other rights of third parties which may result from its use. No license is granted by implication or 
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are 
 * subject to change without notice.
 *
 */

/*! \file bma150_driver.c
    \brief This file contains all function implementations for the BMA150 in linux
    
    Details.
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <asm/uaccess.h>
#include <linux/unistd.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include<linux/platform_device.h>

#include "smb380.h"
#include "smb380calib.h"

#ifdef BMA150_MODULES 
#include "smb380.c"
#include "smb380calib.c"
#endif

#define BMA150_IOC_MAGIC 'B'

#define BMA150_SOFT_RESET				_IO(BMA150_IOC_MAGIC,0)
#define BMA150_SET_RANGE				_IOWR(BMA150_IOC_MAGIC,4, unsigned char)
#define BMA150_GET_RANGE				_IOWR(BMA150_IOC_MAGIC,5, unsigned char)
#define BMA150_SET_MODE					_IOWR(BMA150_IOC_MAGIC,6, unsigned char)
#define BMA150_GET_MODE					_IOWR(BMA150_IOC_MAGIC,7, unsigned char)
#define BMA150_SET_BANDWIDTH			_IOWR(BMA150_IOC_MAGIC,8, unsigned char)
#define BMA150_GET_BANDWIDTH			_IOWR(BMA150_IOC_MAGIC,9, unsigned char)
#define BMA150_SET_WAKE_UP_PAUSE		_IOWR(BMA150_IOC_MAGIC,10,unsigned char)
#define BMA150_GET_WAKE_UP_PAUSE		_IOWR(BMA150_IOC_MAGIC,11,unsigned char)
#define BMA150_SET_LOW_G_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,12,unsigned char)
#define BMA150_GET_LOW_G_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,13,unsigned char)
#define BMA150_SET_LOW_G_COUNTDOWN		_IOWR(BMA150_IOC_MAGIC,14,unsigned char)
#define BMA150_GET_LOW_G_COUNTDOWN		_IOWR(BMA150_IOC_MAGIC,15,unsigned char)
#define BMA150_SET_HIGH_G_COUNTDOWN		_IOWR(BMA150_IOC_MAGIC,16,unsigned char)
#define BMA150_GET_HIGH_G_COUNTDOWN		_IOWR(BMA150_IOC_MAGIC,17,unsigned char)
#define BMA150_SET_LOW_G_DURATION		_IOWR(BMA150_IOC_MAGIC,18,unsigned char)
#define BMA150_GET_LOW_G_DURATION		_IOWR(BMA150_IOC_MAGIC,19,unsigned char)
#define BMA150_SET_HIGH_G_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,20,unsigned char)
#define BMA150_GET_HIGH_G_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,21,unsigned char)
#define BMA150_SET_HIGH_G_DURATION		_IOWR(BMA150_IOC_MAGIC,22,unsigned char)
#define BMA150_GET_HIGH_G_DURATION		_IOWR(BMA150_IOC_MAGIC,23,unsigned char)
#define BMA150_SET_ANY_MOTION_THRESHOLD	_IOWR(BMA150_IOC_MAGIC,24,unsigned char)
#define BMA150_GET_ANY_MOTION_THRESHOLD	_IOWR(BMA150_IOC_MAGIC,25,unsigned char)
#define BMA150_SET_ANY_MOTION_COUNT		_IOWR(BMA150_IOC_MAGIC,26,unsigned char)
#define BMA150_GET_ANY_MOTION_COUNT		_IOWR(BMA150_IOC_MAGIC,27,unsigned char)
#define BMA150_SET_INTERRUPT_MASK		_IOWR(BMA150_IOC_MAGIC,28,unsigned char)
#define BMA150_GET_INTERRUPT_MASK		_IOWR(BMA150_IOC_MAGIC,29,unsigned char)
#define BMA150_RESET_INTERRUPT			_IO(BMA150_IOC_MAGIC,30)
#define BMA150_READ_ACCEL_X				_IOWR(BMA150_IOC_MAGIC,31,short)
#define BMA150_READ_ACCEL_Y				_IOWR(BMA150_IOC_MAGIC,32,short)
#define BMA150_READ_ACCEL_Z				_IOWR(BMA150_IOC_MAGIC,33,short)
#define BMA150_GET_INTERRUPT_STATUS		_IOWR(BMA150_IOC_MAGIC,34,unsigned char)
#define BMA150_SET_LOW_G_INT			_IOWR(BMA150_IOC_MAGIC,35,unsigned char)
#define BMA150_SET_HIGH_G_INT			_IOWR(BMA150_IOC_MAGIC,36,unsigned char)
#define BMA150_SET_ANY_MOTION_INT		_IOWR(BMA150_IOC_MAGIC,37,unsigned char)
#define BMA150_SET_ALERT_INT			_IOWR(BMA150_IOC_MAGIC,38,unsigned char)
#define BMA150_SET_ADVANCED_INT			_IOWR(BMA150_IOC_MAGIC,39,unsigned char)
#define BMA150_LATCH_INT				_IOWR(BMA150_IOC_MAGIC,40,unsigned char)
#define BMA150_SET_NEW_DATA_INT			_IOWR(BMA150_IOC_MAGIC,41,unsigned char)
#define BMA150_GET_LOW_G_HYST			_IOWR(BMA150_IOC_MAGIC,42,unsigned char)
#define BMA150_SET_LOW_G_HYST			_IOWR(BMA150_IOC_MAGIC,43,unsigned char)
#define BMA150_GET_HIGH_G_HYST			_IOWR(BMA150_IOC_MAGIC,44,unsigned char)
#define BMA150_SET_HIGH_G_HYST			_IOWR(BMA150_IOC_MAGIC,45,unsigned char)
#define BMA150_READ_ACCEL_XYZ			_IOWR(BMA150_IOC_MAGIC,46,short)
#define BMA150_READ_TEMPERATURE			_IOWR(BMA150_IOC_MAGIC,47,unsigned char)
#define BMA150_CALIBRATION				_IOWR(BMA150_IOC_MAGIC,48,short)
#define BMA150_IOC_MAXNR				50

//#define BMA150_DEBUG
//#define BMA150_SMBUS

static struct i2c_client *bma150_client = NULL;

struct bma150_data{
	smb380_t smb380;
};

static char bma150_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len);
static char bma150_i2c_read(unsigned char reg_addr, unsigned char *data, unsigned char len);
static void bma150_i2c_delay(unsigned int msec);


/*	i2c delay routine for eeprom	*/
static inline void bma150_i2c_delay(unsigned int msec)
{
	mdelay(msec);
}

/*	i2c write routine for bma150	*/
static inline char bma150_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	s32 dummy;
#ifndef BMA150_SMBUS
	unsigned char buffer[2];
#endif
	if( bma150_client == NULL )	/*	No global client pointer?	*/
		return -1;

	while(len--)
	{
#ifdef BMA150_SMBUS
		dummy = i2c_smbus_write_byte_data(bma150_client, reg_addr, *data);
#else
		buffer[0] = reg_addr;
		buffer[1] = *data;
		dummy = i2c_master_send(bma150_client, (char*)buffer, 2);
#endif
		reg_addr++;
		data++;
		if(dummy < 0)
			return -1;
	}
	return 0;
}

/*	i2c read routine for bma150	*/
static inline char bma150_i2c_read(unsigned char reg_addr, unsigned char *data, unsigned char len) 
{
	s32 dummy;
	if( bma150_client == NULL )	/*	No global client pointer?	*/
		return -1;

	while(len--)
	{        
#ifdef BMA150_SMBUS
		dummy = i2c_smbus_read_byte_data(bma150_client, reg_addr);
		if(dummy < 0)
			return -1;
		*data = dummy & 0x000000ff;
#else
		dummy = i2c_master_send(bma150_client, (char*)&reg_addr, 1);
		if(dummy < 0)
			return -1;
		dummy = i2c_master_recv(bma150_client, (char*)data, 1);
		if(dummy < 0)
			return -1;
#endif
		reg_addr++;
		data++;
	}
	return 0;
}

#ifdef BMA150_MODULE_EXPORT
/* return a short value from a float value*/
// (short)(value * 11.25) ---- 8G mode
static inline short Convert(char range,short value)
{
	if(value == 0)
		return 0;
	value *= 45;
	switch(range)
	{
		case 0:
			if(value & 0x0008)
				value += 0x0008;
			value >>= 4;
			break;
		case 1:
			if(value & 0x0004)
				value += 0x0004;
			value >>= 3;
			break;
		case 2:
			if(value & 0x0002)
				value += 0x0002;
			value >>= 2;
			break;
		default:
			return 0;
	}
	return value;
}

/*	export symbols for other modules	*/
int bma150_get(short* data)
{
	char range;
	smb380acc_t acc;
#ifdef BMA150_DEBUG
		printk(KERN_INFO "%s\n",__FUNCTION__); 
#endif

	if( bma150_client == NULL)
	{
#ifdef BMA150_DEBUG
		printk(KERN_INFO "I2C driver not install\n"); 
#endif
		return -1;
	}

	smb380_get_range(&range);
	smb380_read_accel_xyz(&acc);

	data[0] = Convert(range,acc.x);
	data[1] = Convert(range,acc.y);
	data[2] = Convert(range,acc.z);
	return 0;
}
EXPORT_SYMBOL_GPL(bma150_get);
#endif

/*	read command for BMA150 device file	*/
static ssize_t bma150_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{	
	smb380acc_t acc;	
	int ret;
	if( bma150_client == NULL )
	{
#ifdef BMA150_DEBUG
		printk(KERN_INFO "I2C driver not install\n");
#endif
		return -1;
	}

	smb380_read_accel_xyz(&acc);
#ifdef BMA150_DEBUG
	printk(KERN_INFO "BMA150: X/Y/Z axis: %-8d %-8d %-8d\n" ,
		(int)acc.x, (int)acc.y, (int)acc.z);  
#endif

	if( count != sizeof(acc) )
	{
		return -1;
	}
	ret = copy_to_user(buf,&acc, sizeof(acc));
	if( ret != 0 )
	{
#ifdef BMA150_DEBUG
	printk(KERN_INFO "BMA150: copy_to_user result: %d\n", ret);
      
#endif
          return -1;
	}
	return sizeof(acc);
}

/*	write command for BMA150 device file	*/
static ssize_t bma150_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	if( bma150_client == NULL )
		return -1;
#ifdef BMA150_DEBUG
	printk(KERN_INFO "BMA150 should be accessed with ioctl command\n");
#endif
	return 0;
}

/*	open command for BMA150 device file	*/
static int bma150_open(struct inode *inode, struct file *file)
{
#ifdef BMA150_DEBUG
		printk(KERN_INFO "%s\n",__FUNCTION__); 
#endif

	if( bma150_client == NULL)
	{
#ifdef BMA150_DEBUG
		printk(KERN_INFO "I2C driver not install\n"); 
#endif
		return -1;
	}

	smb380_set_bandwidth(0);		//bandwidth 25Hz
	smb380_set_range(0);			//range +/-2G

#ifdef BMA150_DEBUG
	printk(KERN_INFO "BMA150 has been opened\n");
#endif
	return 0;
}

/*	release command for BMA150 device file	*/
static int bma150_close(struct inode *inode, struct file *file)
{
#ifdef BMA150_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);	
#endif
	return 0;
}


/*	ioctl command for BMA150 device file	*/
static int bma150_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	unsigned char data[6];
	int temp;

#ifdef BMA150_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);	
#endif

	/* check cmd */
	if(_IOC_TYPE(cmd) != BMA150_IOC_MAGIC)	
	{
#ifdef BMA150_DEBUG		
		printk(KERN_INFO "cmd magic type error\n");
#endif
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > BMA150_IOC_MAXNR)
	{
#ifdef BMA150_DEBUG
		printk(KERN_INFO "cmd number error\n");
#endif
		return -ENOTTY;
	}

	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	if(err)
	{
#ifdef BMA150_DEBUG
		printk(KERN_INFO "cmd access_ok error\n");
#endif
		return -EFAULT;
	}
	/* check bam150_client */
	if( bma150_client == NULL)
	{
#ifdef BMA150_DEBUG
		printk(KERN_INFO "I2C driver not install\n"); 
#endif
		return -EFAULT;
	}
	
	/* cmd mapping */

	switch(cmd)
	{
	case BMA150_SOFT_RESET:
		err = smb380_soft_reset();
		return err;

	case BMA150_SET_RANGE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_range(*data);
		return err;

	case BMA150_GET_RANGE:
		err = smb380_get_range(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG			
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_MODE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_mode(*data);
		return err;

	case BMA150_GET_MODE:
		err = smb380_get_mode(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG			
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_BANDWIDTH:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_bandwidth(*data);
		return err;

	case BMA150_GET_BANDWIDTH:
		err = smb380_get_bandwidth(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_WAKE_UP_PAUSE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_wake_up_pause(*data);
		return err;

	case BMA150_GET_WAKE_UP_PAUSE:
		err = smb380_get_wake_up_pause(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_LOW_G_THRESHOLD:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_low_g_threshold(*data);
		return err;

	case BMA150_GET_LOW_G_THRESHOLD:
		err = smb380_get_low_g_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_LOW_G_COUNTDOWN:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_low_g_countdown(*data);
		return err;

	case BMA150_GET_LOW_G_COUNTDOWN:
		err = smb380_get_low_g_countdown(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_HIGH_G_COUNTDOWN:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_high_g_countdown(*data);
		return err;

	case BMA150_GET_HIGH_G_COUNTDOWN:
		err = smb380_get_high_g_countdown(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG			
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_LOW_G_DURATION:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_low_g_duration(*data);
		return err;

	case BMA150_GET_LOW_G_DURATION:
		err = smb380_get_low_g_duration(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_HIGH_G_THRESHOLD:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_high_g_threshold(*data);
		return err;

	case BMA150_GET_HIGH_G_THRESHOLD:
		err = smb380_get_high_g_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_HIGH_G_DURATION:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_high_g_duration(*data);
		return err;

	case BMA150_GET_HIGH_G_DURATION:
		err = smb380_get_high_g_duration(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_ANY_MOTION_THRESHOLD:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_any_motion_threshold(*data);
		return err;

	case BMA150_GET_ANY_MOTION_THRESHOLD:
		err = smb380_get_any_motion_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_ANY_MOTION_COUNT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_any_motion_count(*data);
		return err;

	case BMA150_GET_ANY_MOTION_COUNT:
		err = smb380_get_any_motion_count(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_INTERRUPT_MASK:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_interrupt_mask(*data);
		return err;

	case BMA150_GET_INTERRUPT_MASK:
		err = smb380_get_interrupt_mask(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_RESET_INTERRUPT:
		err = smb380_reset_interrupt();
		return err;

	case BMA150_READ_ACCEL_X:
		err = smb380_read_accel_x((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_READ_ACCEL_Y:
		err = smb380_read_accel_y((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_READ_ACCEL_Z:
		err = smb380_read_accel_z((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_GET_INTERRUPT_STATUS:
		err = smb380_get_interrupt_status(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_LOW_G_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_low_g_int(*data);
		return err;

	case BMA150_SET_HIGH_G_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_high_g_int(*data);
		return err;

	case BMA150_SET_ANY_MOTION_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_any_motion_int(*data);
		return err;

	case BMA150_SET_ALERT_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_alert_int(*data);
		return err;

	case BMA150_SET_ADVANCED_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_advanced_int(*data);
		return err;

	case BMA150_LATCH_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_latch_int(*data);
		return err;

	case BMA150_SET_NEW_DATA_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_new_data_int(*data);
		return err;

	case BMA150_GET_LOW_G_HYST:
		err = smb380_get_low_g_hysteresis(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_LOW_G_HYST:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_low_g_hysteresis(*data);
		return err;

	case BMA150_GET_HIGH_G_HYST:
		err = smb380_get_high_g_hysteresis(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_HIGH_G_HYST:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_high_g_hysteresis(*data);
		return err;

	case BMA150_READ_ACCEL_XYZ:
		err = smb380_read_accel_xyz((smb380acc_t*)data);
		if(copy_to_user((smb380acc_t*)arg,(smb380acc_t*)data,6)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_READ_TEMPERATURE:
		err = smb380_read_temperature(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	/* offset calibration routine */
	case BMA150_CALIBRATION:
		if(copy_from_user((smb380acc_t*)data,(smb380acc_t*)arg,6)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		/* iteration time 10 */
		temp = 10;
		err = smb380_calibrate(*(smb380acc_t*)data, &temp);
		return err;

	default:
		return 0;
	}
}


static const struct file_operations bma150_fops = {
	.owner = THIS_MODULE,
	.read = bma150_read,
	.write = bma150_write,
	.open = bma150_open,
	.release = bma150_close,
	.ioctl = bma150_ioctl,
};

/* May 4th 2009 modified*
 * add miscdevices for bma
 */
static struct miscdevice bma_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "bma150",
	.fops = &bma150_fops,
};

static int bma150_detect(struct i2c_client *client, int kind,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
#ifdef BMA150_DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	strlcpy(info->type, "bma150", I2C_NAME_SIZE);

	return 0;
}

static int bma150_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct bma150_data *data;
	int err = 0;
	char tempvalue;
         struct i2c_adapter *adapter;   

#ifdef BMA150_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif

#if 1
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_INFO "i2c_check_functionality error\n");
		goto exit;
	}

	/* OK. For now, we presume we have a valid client. We now create the
	   client structure, even though we cannot fill it completely yet. */
	if (!(data = kmalloc(sizeof(struct bma150_data), GFP_KERNEL)))
	{
		err = -ENOMEM;
		printk(KERN_INFO "kmalloc error\n");
		goto exit;
	}
	memset(data, 0, sizeof(struct bma150_data));

	i2c_set_clientdata(client, data);
		
	/* read chip id */
	tempvalue = 0;
#ifdef BMA150_SMBUS
	tempvalue = i2c_smbus_read_word_data(client, 0x00);
#else
	i2c_master_send(client, (char*)&tempvalue, 1);
	i2c_master_recv(client, (char*)&tempvalue, 1);
#endif
#endif
	if((tempvalue&0x00FF) == 0x0002)
	{
		printk(KERN_INFO "Bosch Sensortec Device detected!\nBMA150/SMB380 registered I2C driver!\n");
		bma150_client = client;
	}
	else
	{
		printk(KERN_INFO "Bosch Sensortec Device not found, i2c error %d \n", tempvalue);
	//	i2c_detach_client(client);				modified on Apr 27 2009
		bma150_client = NULL;
		err = -1;
		goto exit_kfree;
	}
       // bma150_client = client;
	
	/* May 4th modified for device create*/
	err = misc_register(&bma_device);
	if (err) {
		printk(KERN_ERR "bma150 device register failed\n");
		goto exit_kfree;
	}
	printk(KERN_INFO "bma150 device create ok\n");

	data->smb380.bus_write = bma150_i2c_write;
	data->smb380.bus_read = bma150_i2c_read;
	data->smb380.delay_msec = bma150_i2c_delay;
	smb380_init(&(data->smb380));
	return 0;
exit_kfree:
	kfree(data);
exit:
	return err;
}


static int bma150_remove(struct i2c_client *client)
{
	struct bma150_data *data;
#ifdef BMA150_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif	
	data = i2c_get_clientdata(client);
	bma150_client = NULL;
	misc_deregister(&bma_device);
	kfree(data);
	return 0;
}


static unsigned short normal_i2c[] = { I2C_CLIENT_END};
I2C_CLIENT_INSMOD_1(bma150);

static const struct i2c_device_id bma150_id[] = {
	{ "bma150", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma150_id);

static struct i2c_driver bma150_driver = {	
	.driver = {
		.owner	= THIS_MODULE,	
		.name	= "bma150",
	},
	.class		= I2C_CLASS_HWMON,
	.id_table	= bma150_id,
	.address_data	= &addr_data,
	.probe		= bma150_probe,
	.remove		= bma150_remove,
	.detect		= bma150_detect,
};
static int imapx200_sensor_probe(struct platform_device *pdev)
{
        printk(KERN_EMERG"&&&&& probe");

       // i2c_add_driver(&bma150_driver);
/*******************************************************/
	struct i2c_board_info info;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	int ret;

     /* ret= i2c_add_driver(&bma150_driver);
//	ret = i2c_add_driver(&wm8987_i2c_driver);
	if (ret != 0) {
		dev_err(&pdev->dev, "can't add i2c driver\n");
		return ret;
	}*/
	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = SMB380_I2C_ADDR;  //****
	strlcpy(info.type, "bma150", I2C_NAME_SIZE);

	adapter = i2c_get_adapter(1);
	if (!adapter) {
		printk("*******get_adapter error!\n");
	}
	client = i2c_new_device(adapter, &info);

	//i2c_put_adapter(adapter);
	if (!client) {
		printk("**********put_adapter is error!\n");
	}
/*******************************************************/ 
       ret= i2c_add_driver(&bma150_driver);
//	ret = i2c_add_driver(&wm8987_i2c_driver);
	if (ret != 0) {
		dev_err(&pdev->dev, "can't add i2c driver\n");
		return ret;
	}

}
static struct platform_driver imapx200_sensor_driver=
{
             .probe  = imapx200_sensor_probe,
             .driver =
             {
               .owner = THIS_MODULE,
               .name  = "imapx200_bma150",
             },
};

static int __init BMA150_init(void)
{
#ifdef BMA150_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif
/*      int i;    
    
    i= i2c_add_driver(&bma150_driver);
   printk(KERN_INFO"return is %d\n",i);
    return i;
*/
  platform_driver_register(&imapx200_sensor_driver);
}

static void __exit BMA150_exit(void)
{
	i2c_del_driver(&bma150_driver);
	printk(KERN_ERR "BMA150 exit\n");
}


MODULE_DESCRIPTION("BMA150 driver");
MODULE_LICENSE("GPL");

//module_init(BMA150_init);
late_initcall(BMA150_init);
//module_exit(BMA150_exit);

