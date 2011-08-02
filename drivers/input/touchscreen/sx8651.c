
/*
 * drivers/input/touchscreen/sx8651.c
 *
 * Copyright (c) 2009 Semtech Corp
 *
 * Using code from:
 *  - tsc2007.c
 *      Copyright (c) 2008 Kwangwoo Lee
 *  - ads7846.c
 *      Copyright (c) 2005 David Brownell
 *      Copyright (c) 2006 Nokia Corporation
 *  - corgi_ts.c
 *      Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *      Copyright (C) 2002 MontaVista Software
 *      Copyright (C) 2004 Texas Instruments
 *      Copyright (C) 2005 Dirk Behme
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#define DEBUG 1

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c/sx865x.h>

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/cpufreq.h>
#include <linux/dma-mapping.h>
#include <linux/poll.h>
#include <linux/rtc.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <linux/gpio.h>

/* timeout expires after pen is lifted, no more PENIRQs comming */
#define TS_TIMEOUT		(20 * 1000000)	/* adjust with DLY setting */
//#define TS_TIMEOUT		(20 * 1000000)	/* adjust with DLY setting */
#define TS_POLL_PERIOD          (200 * 1000000) /* battery check time */

/* analog channels */
#define CH_X	0
#define CH_Y	1
#define CH_Z1	2
#define CH_Z2	3
#define CH_AUX	4
#define CH_RX	5
#define CH_RY	6
#define CH_SEQ	7

/* commands */
#define SX8651_WRITE_REGISTER	0x00
#define SX8651_READ_REGISTER	0x40
#define SX8651_SELECT_CH(ch)	(0x80 | ch)
#define SX8651_CONVERT_CH(ch)	(0x90 | ch)
#define SX8651_POWDWN			0xb0	// power down, ignore pen
#define SX8651_PENDET			0xc0	// " " with pen sensitivity
#define SX8651_PENTRG			0xe0	// " " " " and sample channels

/* register addresses */
#define I2C_REG_CTRL0		0x00
#define I2C_REG_CTRL1		0x01
#define I2C_REG_CTRL2		0x02
#define I2C_REG_CTRL3		0x03
#define I2C_REG_CHANMASK	0x04
#define I2C_REG_STAT		0x05
#define I2C_REG_SOFTRESET	0x1f

#define SOFTRESET_VALUE		0xde

/* bits for I2C_REG_STAT */
#define STATUS_CONVIRQ		0x80	// I2C_REG_STAT: end of conversion flag
#define STATUS_PENIRQ		0x40	// I2C_REG_STAT: pen detected

/* sx8651 bits for RegCtrl1 */
#define CONDIRQ         0x20
#define FILT_NONE       0x00    /* no averaging */
#define FILT_3SA        0x01    /* 3 sample averaging */
#define FILT_5SA        0x02    /* 5 sample averaging */
#define FILT_7SA        0x03    /* 7 samples, sort, then average 
                                   of 3 middle samples */

#define TOUCH_SIZE_SUM_SHIFT  3	/* 2=4 samples, 3=8 samples, 4=16 etc */
#define TOUCH_SIZE_SUM_COUNT		(1 << TOUCH_SIZE_SUM_SHIFT)
#define DEFAULT_DELAY 0x09
#define DEFAULT_RATE (0x0F<<4) 
#define RATE_MSK 0x0F
#define RATE_IDX 4

/* bits for register 2, I2CRegChanMsk */
#define CONV_X		0x80
#define CONV_Y		0x40
#define CONV_Z1		0x20
#define CONV_Z2		0x10
#define CONV_AUX	0x08
#define CONV_RX		0x04
#define CONV_RY		0x02

/* power/set delay: lower nibble of CTRL0 register, or SETDLY in CTRL2 */
#define DLY_IMMEDIATE	0x00
#define DLY_1_1US		  0x01
#define DLY_2_2US		  0x02
#define DLY_4_4US		  0x03
#define DLY_8_9US		  0x04
#define DLY_17_8US	  0x05
#define DLY_35_5US	  0x06
#define DLY_71US		0x07
#define DLY_140US		0x08
#define DLY_280US		0x09
#define DLY_570US		0x0a
#define DLY_1_1MS		0x0b
#define DLY_2_3MS		0x0c
#define DLY_4_6MS		0x0d
#define DLY_9MS			0x0e
#define DLY_18MS		0x0f

#define MAX_12BIT		       ((1 << 12) - 1)

/* when changing channel mask, change chan read length appropriately */
//#define CHAN_MASK	(CONV_X | CONV_Y | CONV_Z1 | CONV_Z2 | CONV_RX | CONV_RY )
#define CHAN_MASK	(CONV_X | CONV_Y | CONV_Z1 | CONV_Z2 | CONV_AUX | CONV_RX | CONV_RY )
#define NUM_CHANNELS_SEQ	7

#define SX8651_MAX_DELTA	40

#define _INITIAL_OMAP3EVM_WIDTH_THRESHOLD 	4095 // rx
#define _INITIAL_OMAP3EVM_HEIGHT_THRESHOLD	4095 // ry
#define WIDTH_DIFF_LIMIT	40
#define HEIGHT_DIFF_LIMIT	40
#define HEIGHT_NOISE 25
#define  WIDTH_NOISE 25

#define CALFACTOR_FLAG_NOTOUCHRUNNING 0x01
#define CALFACTOR_FLAG_NOTOUCHCOMPLETED 0x02
#define CALFACTOR_FLAG_DEFAULT 0x00

#define BATTERY_FLAG_CHECKING	0x1
#define BATTERY_FLAG_DEFAULT	0x0

#define SX8651_RXMEAS_MSK 0x07
#define SX8651_RYMEAS_MSK 0x07
#define SX8651_RYMEAS_IDX 3
#define MAX_CODE 4095
#define TOUCH_SIZE 10

#define LARGE_RT 1000

struct ts_event {
	u16     x;
	u16     y;
	u16     z1, z2;
	u16     rx, ry;
	u16     aux;
};

struct ts_event_large {
	u32     x;
	u32     y;
	u32     z1, z2;
	u32     rx, ry;
	u32	aux;
};

typedef struct {
	char pendown : 1;
	char finger2_pressed : 1;
} ts_flags_t;


struct _sx8651_calfactors
{
  char flags;
  u32 rxTotal;
  u32 ryTotal;
  u32 rTouch;
  u32 cTouch;
} typedef sx8651_calfactors, *psx8651_calfactors;

struct sx8651 {
	struct input_dev	*input;
	char		    phys[32];
	struct hrtimer		timer;
	struct hrtimer    	battery_timer;
	struct ts_event	 tc;
	struct ts_event	 prev_tc;
	struct ts_event_large	 sums_tc;
	char	mt_sample_count;

	struct i2c_client       *client;

	struct sx865x_platform_data	*pdata;

	struct workqueue_struct	*ts_workq;
//	struct work_struct	pen_event_work;
	struct work_struct	battery_event_work;
	struct delayed_work pen_event_work;
  sx8651_calfactors calfactors;
  char battery_flags;
	spinlock_t	      lock;

	u16		     model;
	u16		     y_plate_ohms;

	ts_flags_t	flags;
	int		     irq;

	int		     (*get_pendown_state)(void);
	void		    (*clear_penirq)(void);

	int _ns_count;	//tmp debug
	u16 width_threshold;
	u16 height_threshold;
	u16 finger1_count;
	u16 finger2_count;
	char debug;
  char bytesToRead;
	char irq_st;

  char reg0Value; /* memory value of reg0 */

	int32_t  sx8651_delay_time;
	uint32_t sx8651_manual;
	spinlock_t    sx8651_lock;	
};

#define TS_IRQ_DISABLE	(0x1)
#define TS_IRQ_ENABLE	(0x2)

#ifdef CONFIG_MACH_OMAP3EVM

#define OMAP3EVM_XMIN		0x136
#define OMAP3EVM_XMAX		0xe84
#define OMAP3EVM_YMIN		0x0d9
#define OMAP3EVM_YMAX		0xec6

#endif

/* Function Prototypes */
extern struct completion Menu_Button;
extern int imap_iokey_keyled(int, int);
extern int imap_iokey_motor(int, int);
int sx8651_PerformNoTouchCalibration(struct sx8651 *);
int sx8651_EndNoTouchCalibration(struct sx8651 *);


#define SX8651_IDLE		(0x1)
#define SX8651_PEN_DOWN  	(0x2)
#define SX8651_BATTERY_CMD  	(0x3)
#define SX8651_BATTERY_OK  	(0x4)
#define SX8651_BATTERY_DOWN	(0x5)
#define SX8651_BATTERY_BAD	(0x6)

#ifdef CONFIG_BATT_EXTERNAL
#define SX8651_BATTERY_TIME  (10)
int battery_val = 1172;
static void sx8651_battery_check_start(struct sx8651 * ts);
static int  sx8651_battery_check_end(struct sx8651 * ts);
int sx8651_get_adc_val(void)
{
	return battery_val;
}
#endif
unsigned int sx8651_int;
unsigned int irq_no;

static int sx8651_get_resoulce(struct sx8651 * ts, uint32_t type, uint32_t * curst)
{
	uint32_t ret = 0;

	spin_lock(&ts->sx8651_lock);
	
	*curst = ts->sx8651_manual;

	switch(type)
	{
		case SX8651_IDLE:
			if(ts->sx8651_manual == SX8651_BATTERY_BAD)
			{	
				ts->sx8651_manual = SX8651_IDLE;
				ret = 1;
			}
			if(ts->sx8651_manual == SX8651_PEN_DOWN)
			{
				ts->sx8651_manual = SX8651_IDLE;
				ret = 1;
			}
			if(ts->sx8651_manual == SX8651_BATTERY_DOWN)
			{
				ts->sx8651_manual = SX8651_IDLE;
				ret = 1;
			}			
			if(ts->sx8651_manual == SX8651_IDLE)
			{
				ts->sx8651_manual = SX8651_IDLE;
				ret = 1;
			}			
			break;

		case SX8651_PEN_DOWN:
			if(ts->sx8651_manual == SX8651_IDLE)
			{	
				ts->sx8651_manual = SX8651_PEN_DOWN;
				ret = 1;
			}
			if(ts->sx8651_manual == SX8651_PEN_DOWN)
			{	
				ts->sx8651_manual = SX8651_PEN_DOWN;
				ret = 1;
			}
			break;

		case SX8651_BATTERY_CMD:
			if(ts->sx8651_manual == SX8651_IDLE)
			{	
				ts->sx8651_manual = SX8651_BATTERY_CMD;
				ret = 1;
			}
			break;

		case SX8651_BATTERY_OK:
			if(ts->sx8651_manual == SX8651_BATTERY_CMD)
			{	
				ts->sx8651_manual = SX8651_BATTERY_OK;
				ret = 1;
			}
			break;

		case SX8651_BATTERY_DOWN:
			if(ts->sx8651_manual == SX8651_BATTERY_OK)
			{	
				ts->sx8651_manual = SX8651_BATTERY_DOWN;
				ret = 1;
			}
			break;

		case SX8651_BATTERY_BAD:
			if(ts->sx8651_manual == SX8651_BATTERY_CMD)
			{	
				ts->sx8651_manual = SX8651_BATTERY_BAD;
				ret = 1;
			}
			break;
		
		default:
			{
				printk("%s default \n", __func__);
				break;
			}

	}
	
	spin_unlock(&ts->sx8651_lock);
	
	
	if(ret == 0)
	{
#ifdef CONFIG_SX8651_BATTERY_DEBUG 
		printk("type = %d, manual = %d\n", type, ts->sx8651_manual);
#endif
	}

	return ret;
}

#if 0
static int sx8651_set_resoulce(struct sx8651 * ts, uint32_t type)
{
	spin_lock(&ts->sx8651_lock);
	ts->sx8651_manual = type;	
	spin_unlock(&ts->sx8651_lock);

}

static int sx8651_get_resoulce(struct sx8651 * ts)
{
	uint32_t ret;
	spin_lock(&ts->sx8651_lock);
	ret = ts->sx8651_manual;	
	spin_unlock(&ts->sx8651_lock);

	return ret;

}

static int sx8651_set_resoulce_try(struct sx8651 * ts, uint32_t type)
{
	int32_t ret = -1;
	spin_lock(&ts->sx8651_lock);
	if(ts->sx8651_manual == SX8651_IDLE)
	{
		ts->sx8651_manual = type;
		ret = 0;
	}
	spin_unlock(&ts->sx8651_lock);

	return ret == 0 ? ret : ts->sx8651_manual;
}

static void sx8651_release_resoulce(struct sx8651 * ts)
{
	spin_lock(&ts->sx8651_lock);
	ts->sx8651_manual = SX8651_IDLE;
	spin_unlock(&ts->sx8651_lock);
}
#endif

static u32 convertRxRyMeasSettingToValueArray[] =
{
  125,250,625,1250,2500,6250,12500,25000
};
u32 calculateRXRYTotal(u16 rxy,int rxyMeasValue)
{
   return (rxyMeasValue * (MAX_CODE-rxy) / rxy);
}
/*
 * Function Name: sx865x_write_command()
 * Purpose: Easy access for writting commands to SX865X
 * Arguments: struct sx8651 * - pointer to struct data
 *            unsigned char - command to write
 * Returns: 1 - if successful, otherwise 0
*/
static char sx865x_write_command(struct sx8651 *this, 
                                 unsigned char command)
{
  if (this)
  {
	  if (!i2c_smbus_write_byte(this->client, command))
    {
      return 1;
    }
  }
  return 0;
}
/*
 * Function Name: sx865x_write_register()
 * Purpose: Easy access for writting registers to SX865X
 * Description: Writes register and if successful, updates memory values
 *              of registers (RegCtrl0 and RegChanMsk)
 * Arguments: struct sx8651 * - pointer to struct data
 *            unsigned char - register address
 *            unsigned char - register value
 * Returns: 1 - if successful, otherwise 0
*/
static char sx865x_write_register(struct sx8651 *this,
                        unsigned char reg_addr, unsigned char reg_value)
{
  char counter = 0;
  if (this)
  {
	  if (!i2c_smbus_write_byte_data(this->client, reg_addr, reg_value))
    {
#ifdef CONFIG_SX8651_DEBUG
	printk("func = %s, line = %d\n", __func__, __LINE__);
#endif
      if (reg_addr != I2C_REG_CHANMASK)
      {
        if (reg_addr == I2C_REG_CTRL0)
        {
          this->reg0Value = reg_value;
        }
        return 1;
      }
      else
      {
        this->bytesToRead = 0;
        while (counter < 8)
        {	
#ifdef CONFIG_SX8651_DEBUG
	printk("func = %s, line = %d\n", __func__, __LINE__);
#endif
          this->bytesToRead += (((reg_value>>(counter++)) & 0x01) << 1);
        }
        return 1;
      }
    }
	else
	{
		printk("func %s i2c error\n",__func__);
	}
  }
  return 0;
}
/*
 * Function Name: sx865x_read_register()
 * Purpose: Easy access for reading registers to SX865X
 * Description: Reads register and if successful, updates memory values
 *              of registers (RegCtrl0 and RegChanMsk)
 * Arguments: struct sx8651 * - pointer to struct data
 *            unsigned char - register address
 * Returns: register value or a negative number if unsuccessful
*/
static int sx865x_read_register(struct sx8651 *ts, unsigned char reg_addr)
{
	struct i2c_msg msg[2];
	char buf0, buf1;
  char counter = 0;
	int ret;

	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &buf0;
	buf0 = reg_addr | SX8651_READ_REGISTER;
	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &buf1;
	ret = i2c_transfer(ts->client->adapter, msg, 2);
	if (ret < 0) {
		return ret;
  }
	else
  {
    if (reg_addr != I2C_REG_CHANMASK)
    {
        if (reg_addr == I2C_REG_CTRL0)
        {
          ts->reg0Value = ret;
        }
      return buf1;
    }
    else
    {
      ts->bytesToRead = 0;
      while (counter < 8)
      {
        ts->bytesToRead += (((ret>>(counter++)) & 0x01) << 1);
      }
      return buf1;
    }    
  }
}

static ssize_t sx8651_debug_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct sx8651 *ts = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", ts->debug);
}

static ssize_t sx8651_debug_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct sx8651 *ts = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 0, &val))
		return -EINVAL;

	ts->debug = val;

	return count;
}

static DEVICE_ATTR(debug, 0666, sx8651_debug_show, sx8651_debug_store);

static ssize_t sx8651_register_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct sx8651 *ts = dev_get_drvdata(dev);
	int err, addr;

	if (!ts)
		return 0;

	if (strcmp(attr->attr.name, "ctrl0") == 0)
		addr = I2C_REG_CTRL0;
	else if (strcmp(attr->attr.name, "ctrl1") == 0)
		addr = I2C_REG_CTRL1;
	else if (strcmp(attr->attr.name, "ctrl2") == 0)
		addr = I2C_REG_CTRL2;
	else if (strcmp(attr->attr.name, "ctrl3") == 0)
		addr = I2C_REG_CTRL3;
	else
		return 0;

	spin_lock_irq(&ts->lock);

	err = sx865x_read_register(ts, addr);

	spin_unlock_irq(&ts->lock);

	if (err < 0)
		return 0;
	else
		return sprintf(buf, "0x%x\n", err);
}

static ssize_t sx8651_register_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct sx8651 *ts = dev_get_drvdata(dev);
	unsigned long val;
	ssize_t ret;
	int err, addr;

	if (strict_strtoul(buf, 0, &val))
		return -EINVAL;

	if (strcmp(attr->attr.name, "ctrl0") == 0)
		addr = I2C_REG_CTRL0;
	else if (strcmp(attr->attr.name, "ctrl1") == 0)
		addr = I2C_REG_CTRL1;
	else if (strcmp(attr->attr.name, "ctrl2") == 0)
		addr = I2C_REG_CTRL2;
	else if (strcmp(attr->attr.name, "ctrl3") == 0)
		addr = I2C_REG_CTRL3;
	else
		return 0;

	printk(KERN_ERR "writing 0x%x to address 0x%x\n", (int)val, addr);

	spin_lock_irq(&ts->lock);

	err = i2c_smbus_write_byte_data(ts->client, addr, val);
	if (err != 0)
		ret = 0;
	else
		ret = count;

	spin_unlock_irq(&ts->lock);

	return ret;
}

static DEVICE_ATTR(ctrl0, 0666, sx8651_register_show, sx8651_register_store);
static DEVICE_ATTR(ctrl1, 0666, sx8651_register_show, sx8651_register_store);
static DEVICE_ATTR(ctrl2, 0666, sx8651_register_show, sx8651_register_store);
static DEVICE_ATTR(ctrl3, 0666, sx8651_register_show, sx8651_register_store);

static struct attribute *sx8651_attributes[] = {
	&dev_attr_ctrl0.attr,
	&dev_attr_ctrl1.attr,
	&dev_attr_ctrl2.attr,
	&dev_attr_ctrl3.attr,
	&dev_attr_debug.attr,
	NULL,
};

static struct attribute_group sx8651_attr_group = {
	.attrs = sx8651_attributes,
};

/*
 * Function Name: sx8651_send_event()
 * Purpose: Sends events, if any, to user space.
 * Description: This assumes that an interrupt was made and new data has
 *              been read in the touch event struct.
 * Arguments: void* - pointer to struct data
 * Returns: void
*/
static void sx8651_send_event(void *sx)
{
	struct sx8651  *ts = sx;
	struct sx865x_platform_data *pdata = ts->pdata;
	struct input_dev *input = ts->input;
	int finger2_pressed;
	u16	     x, y, z1, z2, rx, ry;
	u32	     rt;
	uint32_t ret;
	uint32_t curst;
	struct timeval t_old, t_cur;
	static a = 0;

	if(!a)
	{
		printk(KERN_ERR "Initializing battery evironment ...\n");
		do_gettimeofday(&t_old);

		a = 1;
	}


        
  if (ts)
  {
    /* ** Copy over the data points to a local variable ** */

#ifdef CONFIG_SX8651_DEBUG	  
	printk("func %s, line = %d\n", __func__, __LINE__);
#endif	 

#ifdef CONFIG_BATT_EXTERNAL
	if ((ts->battery_flags)&BATTERY_FLAG_CHECKING)
	{
		if(sx8651_battery_check_end(ts)==0)
		{
			ret = sx8651_get_resoulce(ts, SX8651_IDLE, &curst);
			if(ret == 0)
			{
				printk("%s, %d\n", __func__, curst);
			}
		}
		battery_val = ts->tc.aux;

		do_gettimeofday(&t_cur);
//		printk(KERN_ERR "[%08d][ADC_VALUE]: %05d (%d)\n", t_cur.tv_sec,
#ifdef CONFIG_SX8651_BATTERY_DEBUG
		printk("battery = %d\n", battery_val);	
#endif
		return 0;
	}
#endif
 
	  x = ts->tc.x;
	  y = ts->tc.y;
	  z1 = ts->tc.z1;
	  z2 = ts->tc.z2;
    rx = ts->tc.rx;
    ry = ts->tc.ry;
    /* ******* */
    if (!(((ts->calfactors).flags)&CALFACTOR_FLAG_NOTOUCHRUNNING))
    { /* This is where normal operation occurs (no calibration running) */
  
    	if (likely(y && z1)) {
    		/* compute touch pressure resistance */
    		rt = z2;
    		rt -= z1;
    		rt *= y;
    		rt *= ts->y_plate_ohms;
    		rt /= z1;
    		rt = (rt + 2047) >> 12;
    	} else{
    		rt = 0;
    	}
    	/* If pressure is extremely large, don't report this set */
    	if (rt >= LARGE_RT) {
    		dev_dbg(&ts->client->dev, "ignored pressure %d\n", rt);
		printk("ignored pressure %d\n",rt);  
    		return;
    	}
    
    	/* this means the pressure calculation points to a pen up.
         Most likely a false last read as the user is lifting the
         instrument */
      if (rt == 0)
    		return;
    
    
      /* If we have not already recorded that we are down, record now */
    	if (!ts->flags.pendown) {
#ifdef CONFIG_SX8651_DEBUG
  	  	printk(KERN_ERR "DOWN\n");
#endif
    		ts->flags.pendown = 1;
    	}
      

#if 0
#if defined(CONFIG_MACH_OMAP3EVM) && !defined(TSLIB)
      /* readjust x and y for the current screen max/mins */
    	x = pdata->x_max -
    		((pdata->x_max * (x - OMAP3EVM_XMIN)) / (OMAP3EVM_XMAX- OMAP3EVM_XMIN));
    	y = pdata->y_max -
    		((pdata->y_max * (y - OMAP3EVM_YMIN)) / (OMAP3EVM_YMAX - OMAP3EVM_YMIN));
#endif /* CONFIG_MACH_OMAP3EVM && !TSLIB */
#endif
    
      /* report the instrument is down */
    	input_report_key(input, BTN_TOUCH, 1);
  
      /* *** Determine whether this is a multi-touch or not *** */
      finger2_pressed = 0;
    	if ( rx > ts->width_threshold) /* check X axis */
      {
        finger2_pressed = 1;  /* Multitouch */
      }
      else /* Single Touch */
      {
    		// lower width threshold as necessary
  	  	if (ts->width_threshold > (rx+WIDTH_NOISE)) 
        {
	  	  	ts->width_threshold = rx + WIDTH_NOISE;
    			printk(KERN_ERR "read rx: %d lowered width_treshold: %d\n", 
                                  rx, ts->width_threshold);
    		}
      }
      if (ry > ts->height_threshold) /* check Y axis */
      {
        finger2_pressed = 1; /* Multitouch */
      }
      else /* Single Touch */
      {
    		// lower height threshold as necessary
    		if (ts->height_threshold > (ry+HEIGHT_NOISE))
        {
          ts->height_threshold = ry + HEIGHT_NOISE;
    			printk(KERN_ERR "read ry: %d lowered height_treshold: %d\n", 
                                    ry, ts->height_threshold);
    		}
      }
      /* ***End Multi-Touch Check*** */
      if (finger2_pressed) 
      { /* Have more than one finger on the screen */
    		int diff_width, diff_height;
    		char do_report = 0;
        /* Check if we have already started dualtouch*/
    		if (ts->flags.finger2_pressed) {
#ifdef CONFIG_SX8651_DEBUG	  
	printk("func %s, line = %d\n", __func__, __LINE__);
#endif	  
    			ts->finger1_count = 0;
    			diff_width = abs(rx - ts->prev_tc.rx);
    			diff_height = abs(ry - ts->prev_tc.ry);
    			if (diff_width > WIDTH_DIFF_LIMIT) {
            ts->flags.finger2_pressed = 0; /* ignore this point */
    			}
    			if (diff_height > HEIGHT_DIFF_LIMIT) {
            ts->flags.finger2_pressed = 0; /* ignore this point */
    			}
          /* regardless of any jumps, check this with the next point */
         	ts->prev_tc.rx = rx;
    			ts->prev_tc.ry = ry;
    			ts->prev_tc.x = x;
    			ts->prev_tc.y = y;
          if (!ts->flags.finger2_pressed)
          {
            ts->flags.finger2_pressed = 1;
            return;
          }
    
    			/* *** averaging ***/
    			ts->sums_tc.rx += rx;
    			ts->sums_tc.ry += ry;
    			ts->sums_tc.x += x;
    			ts->sums_tc.y += y;
    			if ((++ts->mt_sample_count) == TOUCH_SIZE_SUM_COUNT) {
    			  ts->finger2_count++;
		    		do_report = 1;
    				rx = ts->sums_tc.rx >> TOUCH_SIZE_SUM_SHIFT;
    				ry = ts->sums_tc.ry >> TOUCH_SIZE_SUM_SHIFT;
    				x = ts->sums_tc.x >> TOUCH_SIZE_SUM_SHIFT;
    				y = ts->sums_tc.y >> TOUCH_SIZE_SUM_SHIFT;
    
    				ts->sums_tc.rx = 0;
    				ts->sums_tc.ry = 0;
    				ts->sums_tc.x = 0;
    				ts->sums_tc.y = 0;
    				ts->mt_sample_count = 0;
    			}
    		} else {
#ifdef CONFIG_SX8651_DEBUG	  
	printk("func %s, line = %d\n", __func__, __LINE__);
#endif	  
    			do_report = 0;
    			ts->mt_sample_count = 0;
    			ts->sums_tc.rx = 0;
    			ts->sums_tc.ry = 0;
    			ts->sums_tc.x = 0;
    			ts->sums_tc.y = 0;
    			ts->prev_tc.rx = rx;
    			ts->prev_tc.ry = ry;
    			ts->prev_tc.x = x;
    			ts->prev_tc.y = y;
    		printk(KERN_ERR "finger2 DOWN\n");
//              "finger2 DOWN %d RX %d RY %d\n", 
//              ts->finger1_count, ts->tc.rx,ts->tc.ry);
    		  ts->flags.finger2_pressed = 1;
    		}
    		if (do_report) {
          /**********************************/
      		if (rx < ts->width_threshold)
      			rx = 0;
      		else
    	  		rx -= ts->width_threshold;
    
    		  if (ry < ts->height_threshold)
    			  ry = 0;
    		  else
    			  ry -= ts->height_threshold;
          /**********************************/
     		  if (ts->debug)
    		    printk(KERN_ERR "1=%d %d	2=%d %d=\n",
                                              x-rx, 
                                              y-ry, 
                                              x+rx, 
                                              y+ry);
          /**********************************/
          /* X1/Y1 */
  	  		input_report_abs(input, ABS_MT_TOUCH_MAJOR, rt);
    			input_report_abs(input, ABS_MT_POSITION_X, 
                                        x - rx);
    			input_report_abs(input, ABS_MT_POSITION_Y, 
                                        y - ry);
    			input_mt_sync(input);
          /* X2/Y2 */
    			input_report_abs(input, ABS_MT_TOUCH_MAJOR, rt);
    			input_report_abs(input, ABS_MT_POSITION_X, 
                                        x + rx);
    			input_report_abs(input, ABS_MT_POSITION_Y, 
                                        y + ry);
    			input_mt_sync(input);
    			input_sync(input);
          /**********************************/
    		}
    
    		/* end double-touch */
    	} else { 
    		/* single-touch */
#ifdef CONFIG_SX8651_DEBUG	  
	printk("func %s, line = %d\n", __func__, __LINE__);
#endif	  
    		finger2_pressed = 0;
    		ts->finger1_count++;
    
    		if (ts->debug)
    			printk(KERN_ERR "1=%d %d\n", x , y);
    
    
    		if (ts->flags.finger2_pressed) {
    			printk(KERN_ERR 
              "finger2 UP %d RX %d RY %d\n", 
              ts->finger2_count, ts->tc.rx,ts->tc.ry);
    		} // ...if finger2 was pressed
        else /* ignore the first reading after lifting finger */
        {
#ifdef CONFIG_SX8651_DEBUG
		  printk("figer 1 is touched, x = %d, y = %d, rt = %d\n",x, y, rt);
#endif
    		  input_report_abs(input, ABS_MT_TOUCH_MAJOR, rt);
    		  input_report_abs(input, ABS_MT_POSITION_X, x);
    		  input_report_abs(input, ABS_MT_POSITION_Y, y);
    		  input_mt_sync(input);
    		  input_sync(input);
        }
    		ts->flags.finger2_pressed = 0;
    		ts->finger2_count = 0;
    	}
    	dev_dbg(&ts->client->dev, "point(%4d,%4d), pressure (%4u)\n",
    		x, y, rt);
    } /* if not in calibration */
    else
    {
#ifdef CONFIG_SX8651_DEBUG	  
	printk("func %s, line = %d\n", __func__, __LINE__);
#endif	  
      if (sx8651_PerformNoTouchCalibration(ts))
      {
        sx8651_EndNoTouchCalibration(ts);
      }
    }
  }  /* if (ts) */
} /* end function */

/* Name: sx8651_StartNoTouchCalibration()
 * Purpose: Start No Touch calibration
 * Description: changes device to manual mode and performs selct/convert
 *        on RX/RY. We select on only RX to bypass a touch being needed.
*/
int sx8651_StartNoTouchCalibration(struct sx8651 *this)
{
#ifdef DEBUG
 	printk(KERN_ERR "StartNoTouchCalibration(%x)\n",this);
#endif
  if (this)
  {
    /* Go into Manual Mode */
    if (sx865x_write_command(this,SX8651_POWDWN))
    {
      /* Finish going into Manual Mode */
      if (sx865x_write_register(this,I2C_REG_CTRL0,0))
      {
        /* Enable ONLY RX / RY */
        if (sx865x_write_register(this,I2C_REG_CHANMASK,CONV_RX | CONV_RY))
        {
          /* Perform Select Command, this will in effect disable pen
             detection. */
          if (sx865x_write_command(this,SX8651_SELECT_CH(CH_RX))) 
          {
            /* Perform Convert Command */
            if (sx865x_write_command(this,SX8651_CONVERT_CH(CH_SEQ)))
            {
              /* Set flag that we are in a notouch calibration */
              this->calfactors.flags |= CALFACTOR_FLAG_NOTOUCHRUNNING;
#ifdef DEBUG
  	          printk(KERN_ERR "\tStarted NoTouchCalibration\n");
#endif
              return 1;
            }
          }
        }
      }
    }
  }
#ifdef DEBUG
 	    printk(KERN_ERR "Failed\n");
#endif
  return 0;
}
/*
  Name: sx8651_PerformNoTouchCalibration()
  Purpose: Use data from channel read in argument to calculate out some
            approximate values for settings in the registers
  Argument: struct sx8651 * - this pointer
  Returns: 0 - fail, 1 - success
*/
int sx8651_PerformNoTouchCalibration(struct sx8651 *this)
{
  int regValue = 0;
  int rxMeasValue = 0;
  int ryMeasValue = 0;
  int counter = 0;
  int rPenDetectReg = 0;
  u32 rPenDetectValue = 0;
  u32 tempA = 0;
  u32 tempB = 0;
#ifdef DEBUG
 	printk(KERN_ERR "PerformNoTouchCalibration(%x)\n",this);
#endif
  if (this)
  {
    /* Read Register 3 to calculate out an rx and an ry meas value */
    if ((regValue = sx865x_read_register(this,I2C_REG_CTRL3)) >= 0)
    {
      /* *Begin***RX/RYMeas and RX/RYTotal Calculation*********** */
      /* Convert register bits to human readable value for Rx/RY meas */
      rxMeasValue = 
        convertRxRyMeasSettingToValueArray[regValue&SX8651_RXMEAS_MSK];
      ryMeasValue =
        convertRxRyMeasSettingToValueArray[(regValue>>SX8651_RYMEAS_IDX)
                                                     &SX8651_RYMEAS_MSK];
#ifdef DEBUG
 	    printk(KERN_ERR "I2C_REG_CTRL3: %x RXMEAS: %d RYMEAS: %d\n",
                          regValue,rxMeasValue,ryMeasValue);
#endif
      /* Calculate Approximate rxtotal and rytotal */
      this->calfactors.rxTotal =
              calculateRXRYTotal(this->tc.rx,rxMeasValue);
      this->calfactors.ryTotal =
              calculateRXRYTotal(this->tc.ry,ryMeasValue);
#ifdef DEBUG
 	    printk(KERN_ERR "RXtotal: %d RYtotal: %d\n",
                  this->calfactors.rxTotal,this->calfactors.ryTotal);
#endif
      /* Adjust the rxmeas and rymeas values accordingly */
      for(rxMeasValue = 0; rxMeasValue < 6; rxMeasValue++)
      {
        if (this->calfactors.rxTotal < 
                        convertRxRyMeasSettingToValueArray[rxMeasValue])
        {
          break;
        }
      }
      for(ryMeasValue = 0; ryMeasValue < 6; ryMeasValue++)
      {
        if (this->calfactors.ryTotal < 
                        convertRxRyMeasSettingToValueArray[ryMeasValue])
        {
          break;
        }
      }
#ifdef DEBUG
 	    printk(KERN_ERR "Adjusting I2C_REG_CTRL3 RXMEAS: %d RYMEAS: %d",
                        rxMeasValue,ryMeasValue);
 	    printk(KERN_ERR "- Values RXMEAS: %d RYMEAS: %d\n",
                        convertRxRyMeasSettingToValueArray[rxMeasValue],
                        convertRxRyMeasSettingToValueArray[ryMeasValue]);
#endif
      if (sx865x_write_register(this,I2C_REG_CTRL3,
                    (SX8651_RXMEAS_MSK & (rxMeasValue)) | 
                    ((SX8651_RYMEAS_MSK&(ryMeasValue))
                                      <<SX8651_RYMEAS_IDX) ))
      {
        /*  Update rxtotal and rytotal to include this offset */
        this->calfactors.rxTotal += 
                  convertRxRyMeasSettingToValueArray[rxMeasValue];
        this->calfactors.ryTotal += 
                  convertRxRyMeasSettingToValueArray[ryMeasValue];
#ifdef DEBUG
 	    printk(KERN_ERR "Adjusting RXtotal: %d RYtotal: %d\n",
                  this->calfactors.rxTotal,this->calfactors.ryTotal);
#endif
        /* *End***RX/RYMeas and RX/RYTotal Calculation*********** */
#ifdef DEBUG
 	    printk(KERN_ERR "Success\n");
#endif
        return 1;
      }
    }
  }
#ifdef DEBUG
  printk(KERN_ERR "Failed\n");
#endif
  return 0;
}

int sx8651_EndNoTouchCalibration(struct sx8651 *this)
{
#ifdef DEBUG
 	printk(KERN_ERR "EndNoTouchCalibration(%x)\n",this);
#endif
  if (this)
  {
    /*remove calrunning*/
    this->calfactors.flags &= ~(CALFACTOR_FLAG_NOTOUCHRUNNING);
    /* Enable Default Channels */
    if (sx865x_write_register(this,I2C_REG_CHANMASK, CHAN_MASK))
    {
      /* Go into Automatic Mode */
      if (sx865x_write_register(this,I2C_REG_CTRL0,
                                    DEFAULT_RATE | DEFAULT_DELAY))
      { 
        /*mark cal complete*/
        this->calfactors.flags |= CALFACTOR_FLAG_NOTOUCHCOMPLETED;  
#ifdef DEBUG
 	      printk(KERN_ERR "Success\n");
#endif

#ifdef CONFIG_BATT_EXTERNAL
	this->sx8651_delay_time = SX8651_BATTERY_TIME;
#endif
        return 1;
      }
    }
  }
#ifdef DEBUG
 	printk(KERN_ERR "Failed\n");
#endif
  return 0;
}
/*
 * Function Name: sx8651_read_values()
 * Purpose: Reads touch screen channel data from the device.
 * Description: This assumes that an interrupt was made.
 * Arguments: struct sx8651 * - pointer to struct data
 * Returns: 0 - successful, -1 - unsuccessful
*/
static int sx8651_read_values(struct sx8651 *tsc)
{
	s32 data = 0;
	u16 vals[NUM_CHANNELS_SEQ+1];	// +1 for last dummy read
	int length = 0;
	int i = 0;
  int ret = 0;
	/* The protocol and raw data format from i2c interface:
	 * S Addr R A [DataLow] A [DataHigh] A (repeat) NA P
	 * Where DataLow has (channel | [D11-D8]), DataHigh has [D7-D0].
	 */

#ifdef CONFIG_SX8651_DEBUG
	printk("tsc->bytesToRead = %d\n", tsc->bytesToRead);
#endif
	length = i2c_master_recv(tsc->client, (char *)vals, tsc->bytesToRead);
	if (likely(length == tsc->bytesToRead)) {
		length >>= 1;
		for (i = 0; i < length; i++) {
			u16 ch;
			data = swab16(vals[i]);
			if (unlikely(data & 0x8000)) {
				printk(KERN_ERR "hibit @ %d\n", i);
				ret = -1;
				continue;
			}
			ch = data >> 12;
			if (ch == CH_X) {
				tsc->tc.x = data & 0xfff;
			} else if (ch == CH_Y) {
				tsc->tc.y = data & 0xfff;
			} else if (ch == CH_Z1) {
				tsc->tc.z1 = data & 0xfff;
			} else if (ch == CH_Z2) {
				tsc->tc.z2 = data & 0xfff;
			} else if (ch == CH_RX) {
				tsc->tc.rx = data & 0xfff;
			} else if (ch == CH_RY) {
				tsc->tc.ry = data & 0xfff;
			} else if (ch == CH_AUX){
				tsc->tc.aux = data & 0xfff;
#ifdef CONFIG_BATT_EXTERNAL
				battery_val = tsc->tc.aux;
	//		printk("****************************CH_AUX = %d\n", battery_val);	
				//if(batt_init_ok ==1)
				//supply_timer_func();			

		//		printk("CH_AUX = %x\n", battery_val);	
#endif
			} else {
				printk(KERN_ERR "? %d %x\n", ch, data & 0xfff);
				ret = -1;
			}

		}
#ifdef CONFIG_SX8651_DEBUG
			printk("x = %d, y = %d, z1 = %d, z2 = %d\n",tsc->tc.x, tsc->tc.y, tsc->tc.z1, tsc->tc.z2);
#endif
	} else {
		printk(KERN_ERR "%d = recv()\n", length);
	//	ret = -1;
	}
	return ret;
}

#ifdef CONFIG_SX8651_KEY
static int back_key_st;
static int menu_key_st;
static int home_key_st;
static int search_key_st;

static void sx8651_check_key(struct sx8651 *ts)
{
	if(back_key_st == 1)
	{
		input_event(ts->input, EV_KEY, KEY_BACK, 0);
		back_key_st = 0;
#ifdef CONFIG_SX8651_KEY_DEBUG
		printk("KEY_BACK UP\n");
#endif
		imap_iokey_keyled(1, 10000);
		imap_iokey_motor(1, 40);
	}
	if(menu_key_st == 1)
	{
		input_event(ts->input, EV_KEY, KEY_MENU, 0);
		menu_key_st = 0;
#ifdef CONFIG_SX8651_KEY_DEBUG
		printk("KEY_MENU UP\n");
#endif
#ifdef CONFIG_HDMI_OUTPUT_SUPPORT
			complete(&Menu_Button);
#endif
		imap_iokey_keyled(1, 10000);
		imap_iokey_motor(1, 40);
	}
	if(home_key_st == 1)
	{
		input_event(ts->input, EV_KEY, KEY_HOME, 0);
		home_key_st = 0;
#ifdef CONFIG_SX8651_KEY_DEBUG
		printk("KEY_HOME UP\n");
#endif
		imap_iokey_keyled(1, 10000);
		imap_iokey_motor(1, 40);
	}
	if(search_key_st == 1)
	{
		input_event(ts->input, EV_KEY, KEY_SEARCH, 0);
		search_key_st = 0;
#ifdef CONFIG_SX8651_KEY_DEBUG
		printk("KEY_SEARCH UP\n");
#endif
		imap_iokey_keyled(1, 10000);
		imap_iokey_motor(1, 40);
	}

}

static void sx8651_report_key_logic(struct sx8651 *ts, int y)
{
	if(y > 20 && y < 140)
	{
                                if(search_key_st == 0)
                                {
                                input_event(ts->input, EV_KEY, KEY_SEARCH, 1);
                                search_key_st = 1;
#ifdef CONFIG_SX8651_KEY_DEBUG
				printk("KEY_SEARCH DOWN LOGIC\n");
#endif
                                }
	}

	if(y > 160 && y < 280)
	{
                                if(back_key_st == 0)
                                {
                                input_event(ts->input, EV_KEY, KEY_BACK, 1);
                                back_key_st = 1;
#ifdef CONFIG_SX8651_KEY_DEBUG
				printk("KEY_BACK DOWN LOGIC\n");
#endif
                                }
	}

	if(y > 300 && y < 420)
	{
                                if(menu_key_st == 0)
                                {
                                input_event(ts->input, EV_KEY, KEY_MENU, 1);
                                menu_key_st = 1;
#ifdef CONFIG_SX8651_KEY_DEBUG
				printk("KEY_MENU DOWN LOGIC\n");
#endif
                                }
	}

	if(y > 440 && y < 560)
	{
                                if(home_key_st == 0)
                                {
                                input_event(ts->input, EV_KEY, KEY_HOME, 1);
                                home_key_st = 1;
#ifdef CONFIG_SX8651_KEY_DEBUG
				printk("KEY_HOME DOWN LOGIC\n");
#endif
                                }
	}	

}

static void sx8651_report_key(struct sx8651 *ts)
{
                        if(ts->tc.x >= 2200 && ts->tc.x <= 2600)
                        {
                                if(back_key_st == 0)
                                {
                                input_event(ts->input, EV_KEY, KEY_BACK, 1);
                                back_key_st = 1;
#ifdef CONFIG_SX8651_KEY_DEBUG
				printk("KEY_BACK DOWN\n");
#endif
                                }
                        }

                        if(ts->tc.x >= 1400 && ts->tc.x <= 1800)
                        {
                                if(menu_key_st == 0)
                                {
                                input_event(ts->input, EV_KEY, KEY_MENU, 1);
                                menu_key_st = 1;
#ifdef CONFIG_SX8651_KEY_DEBUG
				printk("KEY_MENU DOWN\n");
#endif
                                }
                        }

                        if(ts->tc.x >= 3000 && ts->tc.x <= 3400)
                        {
                                if(search_key_st == 0)
                                {
                                input_event(ts->input, EV_KEY, KEY_SEARCH, 1);
                                search_key_st = 1;
#ifdef CONFIG_SX8651_KEY_DEBUG
				printk("KEY_SEARCH DOWN\n");
#endif
                                }
                        }

                        if(ts->tc.x >= 600 && ts->tc.x <= 1000)
                        {
                                if(home_key_st == 0)
                                {
                                input_event(ts->input, EV_KEY, KEY_HOME, 1);
                                home_key_st = 1;
#ifdef CONFIG_SX8651_KEY_DEBUG
				printk("KEY_HOME DOWN\n");
#endif
                                }
                        }

}


#define IX_ISFIRSTTIME		"/data/data/isFirstTime"
#define IX_POINTERCAL         "/data/misc/tscal/pointercal"
static uint32_t file_read_st;
static uint32_t data_read_st;
static int __kread_file_t(const char *s, uint8_t *buf, uint32_t len)
{
        struct file *fp;
    mm_segment_t old_fs;
    loff_t pos = 0;
        size_t ret, readed = 0, read_len;


    fp = filp_open(s, O_RDONLY, 0);
        if(IS_ERR(fp))
        {
                printk(KERN_ERR "%s: failed to open file %s\n",
                   __func__, s);
                return -EFAULT;
        }


    old_fs = get_fs();
    set_fs(KERNEL_DS);

        {
                int i;
                ret = vfs_read(fp, buf, len, &pos);

                if(ret < len)
                {
                        printk(KERN_ERR "%s: file end before demonded bytes.\n",
                           __func__);
                }

        }

    set_fs(old_fs);

    filp_close(fp, 0);

        return ret;
}


#endif 


#if 0

static enum hrtimer_restart sx8651_timer_handler(struct hrtimer *handle)
{
	struct sx8651 *ts = container_of(handle, struct sx8651, timer);
	struct input_dev *input = ts->input;
	uint32_t ret;
	uint32_t curst;
#ifdef CONFIG_SX8651_DEBUG
	printk("sx8651_timer_handler\n");
#endif
	spin_lock(&ts->lock);
	if (unlikely(ts->get_pendown_state())) {
		/* the PENIRQ is low,
		 * meaning the interrupt has not yet been serviced */
		ts->_ns_count++;
		hrtimer_forward_now(&ts->timer, ktime_set(0, TS_TIMEOUT));
		/* kernel has been abused: work queue dropped? */
		printk(KERN_ERR "F %d\n", ts->_ns_count);
		if (ts->_ns_count > 0) {	
			/* queue_work(ts->ts_workq, &ts->pen_event_work); */
			queue_delayed_work(ts->ts_workq, &ts->pen_event_work,4);
			ts->_ns_count = 0;
		}
	  spin_unlock(&ts->lock);
		return HRTIMER_RESTART;
	}
	/* This timer expires after PENIRQs havent been coming in for some time.
	 * It means that the pen is now UP. */

#ifdef CONFIG_SX8651_KEY
	sx8651_check_key(ts);
#endif

#if 0
	input_report_key(input, BTN_TOUCH, 0);
	//single-touch only: input_report_abs(input, ABS_PRESSURE, 0);
	input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0);
	input_mt_sync(input);
	input_sync(input);
	ts->flags.pendown = 0;
	ts->flags.finger2_pressed = 0;
#endif
#ifdef CONFIG_BATT_EXTERNAL
	ret = sx8651_get_resoulce(ts, SX8651_IDLE, &curst);
	if(curst != SX8651_PEN_DOWN && curst != SX8651_BATTERY_DOWN && curst != SX8651_IDLE)
	{
#ifdef CONFIG_SX8651_BATTERY_DEBUG
		printk("func %s ,wait for aux data ready %d\n", __func__, curst);
#endif
	}

	if((ts->battery_flags) & BATTERY_FLAG_CHECKING)
	{
		if(sx8651_battery_check_end(ts) == 0)
		{
        		ret = sx8651_get_resoulce(ts, SX8651_IDLE, &curst);
        		if(ret == 0)
        		{
                		printk("%s, %d, %d\n",__func__, __LINE__, curst);
        		}
		}
	}
#endif

#ifdef CONFIG_SX8651_DEBUG
	printk(KERN_ERR "UP\n");
#endif
	spin_unlock(&ts->lock);
	return HRTIMER_NORESTART;
}
#endif


static enum hrtimer_restart sx8651_timer_handler(struct hrtimer *handle)
{
	struct sx8651 *ts = container_of(handle, struct sx8651, timer);
	struct input_dev *input = ts->input;
	uint32_t ret;
	uint32_t curst;
	
	spin_lock(&ts->lock);
	if (!(ts->get_pendown_state())) {
	
	input_report_key(input, BTN_TOUCH, 0);
	//single-touch only: input_report_abs(input, ABS_PRESSURE, 0);
	input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0);
	input_mt_sync(input);
	input_sync(input);
	ts->flags.pendown = 0;
	ts->flags.finger2_pressed = 0;
#ifdef CONFIG_SX8651_KEY
	sx8651_check_key(ts);
#endif

#ifdef CONFIG_BATT_EXTERNAL
	if((ts->battery_flags) & BATTERY_FLAG_CHECKING)
	{
		if(sx8651_battery_check_end(ts) == 0)
		{
        		ret = sx8651_get_resoulce(ts, SX8651_IDLE, &curst);
        		if(ret == 0)
        		{
                		printk("%s, %d, %d\n",__func__, __LINE__, curst);
        		}
		}
	}
	ret = sx8651_get_resoulce(ts, SX8651_IDLE, &curst);
#endif
#ifdef CONFIG_SX8651_DEBUG
	printk("PEN UP 1\n");
#endif

	}
	spin_unlock(&ts->lock);
	
	enable_irq(ts->irq);

	return HRTIMER_NORESTART;
	



}




static void sx8651_pen_irq_worker(struct work_struct *work)
{
	int reg_val;
	struct sx8651 *ts = container_of(work, struct sx8651, pen_event_work);
#ifdef CONFIG_SX8651_DEBUG
	printk("sx8651_pen_irq_worker\n");
#endif

#ifdef CONFIG_SX8651_KEY
	uint32_t pointercal_buf[125];
	uint8_t * cp;
	uint8_t *p;
	int32_t len;
	uint32_t i=  0;
	uint32_t ret = 0;
	uint32_t reverse = 0;

	int32_t lefttopx, lefttopy, rightbottomx, rightbottomy;  //touch screen x, y
	int32_t lefttopX, lefttopY, rightbottomX, rightbottomY;  //lcd screen x, y
	static int32_t ratiox, ratioy;
	static int32_t KX, KY;
	static int32_t pointercal_num = 0;
	static int32_t pointercal_cnt[20];
	int32_t ts_x, ts_y;
	int32_t lcdx, lcdy;
	static int32_t lcdx_max, lcdy_max; 

#if 0
	//test 
	lefttopX = 50;
	lefttopY = 50;
	rightbottomX = 750;
	rightbottomY = 430;

	lefttopx = 3900;
	lefttopy = 3800;
	rightbottomx = 500;
	rightbottomy = 280;
#endif


	if(file_read_st == 0)
	{

		len = __kread_file_t(IX_ISFIRSTTIME, (uint8_t *)pointercal_buf, 1);
		if(len < 0)
		{
#if CONFIG_SX8651_KEY_DEBUG
			printk("data/data/isFirstTime is not existed\n");
#endif
		}
		else
		{
		for(i=0;i<10;i++)
			pointercal_buf[i] = 0;	
		len = __kread_file_t(IX_POINTERCAL, (uint8_t *)pointercal_buf, 500);
		if(len < 0)
		{
#if CONFIG_SX8651_KEY_DEBUG
			printk("data/misc/tscal/pointercal read err\n");
#endif
		}
		else
		{
#if CONFIG_SX8651_KEY_DEBUG
			printk("len = %d, %x\n",len,pointercal_buf);
#endif
			cp = (uint8_t *)pointercal_buf;	
			while(len > 0)
			{	
				if(*cp == '-')
				{		
					cp += 1;
					reverse = 1;
					len -= 1;
				}
			
				ret = simple_strtoul(cp, &p, 10);
				if(reverse == 1)
				{	
					ret = -ret;
					reverse = 0;
				}
				len -= (int32_t)(p - cp + 1);
				cp = p + 1;
				if(pointercal_num>=19)
					break;
				pointercal_cnt[pointercal_num] = ret;
				pointercal_num++;
#if CONFIG_SX8651_KEY_DEBUG
				printk("%d, %x, %d\n",ret, p, len);
#endif
			}
			file_read_st = 1;
#if CONFIG_SX8651_KEY_DEBUG
			printk("pointercal_num = %d\n",pointercal_num);
#endif			
		}
		
		if(pointercal_num >= 17)
		{
			lefttopX = pointercal_cnt[9];
			lefttopY = pointercal_cnt[10];
			rightbottomX = pointercal_cnt[11];
			rightbottomY = pointercal_cnt[12];
			/*it need to check hardware X+ X- Y+ Y- */
			lefttopx = pointercal_cnt[14];
			lefttopy = pointercal_cnt[13];
			rightbottomx = pointercal_cnt[16];
			rightbottomy = pointercal_cnt[15];
			 
			lcdx_max = pointercal_cnt[7];
			lcdy_max = pointercal_cnt[8];

			ratiox = ((rightbottomx - lefttopx) * 100000)/(rightbottomX - lefttopX);
			ratioy = ((rightbottomy - lefttopy) * 100000)/(rightbottomY - lefttopY);

			KX = lefttopx*100000 - (lefttopX * ratiox);
			KY = lefttopy*100000 - (lefttopY * ratioy);
			data_read_st = 1;
#if CONFIG_SX8651_KEY_DEBUG
			printk("ratiox = %d, ratioy = %d, KX = %d, KY = %d, lcdx_max = %d, lcdy_max = %d\n", ratiox, ratioy, KX, KY, lcdx_max, lcdy_max);
#endif
		}
		}


	}
#endif	
	
	ts->_ns_count = 0;
	/* the pen is down */
	//enable_irq(ts->irq);

	

	if (likely(sx8651_read_values(ts) == 0)) {
		/* valid data was read in */
#ifdef CONFIG_BATT_EXTERNAL
		if ((ts->battery_flags)&BATTERY_FLAG_CHECKING)
		{
#ifdef CONFIG_SX8651_BATTERY_DEBUG
			printk("func %s, battery checking\n",__func__);
#endif
			sx8651_send_event(ts);
			goto finish_ts_event;
		}
#endif


#ifdef  CONFIG_SX8651_KEY
		if (((ts->calfactors).flags)&CALFACTOR_FLAG_NOTOUCHRUNNING)
		{
#ifdef CONFIG_SX8651_DEBUG
			printk("func %s, calfactor_flag_notouchruning\n",__func__);
#endif
			sx8651_send_event(ts);
			goto finish_ts_event;		
		}

		if(data_read_st == 1)
		{
			/*it need to check hardware X+ X- Y+ Y- */
			ts_x = ts->tc.y;
			ts_y = ts->tc.x;

			lcdx = ((ts_x*100000 - KX) / ratiox);
#if 0
			printk("ts_x = %d, ts_y = %d, lcdx = %d, lcdy = %d\n", ts_x, ts_y, lcdx, lcdy);
#endif
			if(lcdx > lcdx_max)
			{
				lcdy = ((ts_y*100000 - KY) / ratioy);
				sx8651_report_key_logic(ts, lcdy);
			}
			else
			{
				sx8651_send_event(ts);
			}
		}
		else
		{
			if(ts->tc.y < 450)
			{

				sx8651_report_key(ts);
			}
			else
#endif
			{
				sx8651_send_event(ts);
			}
#ifdef  CONFIG_SX8651_KEY
		}
#endif
	}else {
		printk(KERN_ERR "fail\n");
	}

#if 0
	mdelay(1);
	if (!(ts->get_pendown_state()))
	{
		input_report_key(ts->input, BTN_TOUCH, 0);
		//single-touch only: input_report_abs(input, ABS_PRESSURE, 0);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 0);
		input_mt_sync(ts->input);
		input_sync(ts->input);
		ts->flags.pendown = 0;
		ts->flags.finger2_pressed = 0;
		printk("PEN UP\n");	
	}
#endif

finish_ts_event:
	/* this timer upon expiration will indicate pen UP */
	hrtimer_start(&ts->timer, ktime_set(0, TS_TIMEOUT), HRTIMER_MODE_REL);

#if 0
	if(ts->irq_st == TS_IRQ_DISABLE)
	{	
		enable_irq(ts->irq);
		ts->irq_st = TS_IRQ_ENABLE;
	}
#endif

//	enable_irq(ts->irq);

}

static irqreturn_t sx8651_irq(int irq, void *handle)
{
	struct sx8651 *ts = handle;
	unsigned long flags;
	uint32_t ret;
	uint32_t curst;
	

	if (ts->get_pendown_state()) {
#ifdef CONFIG_SX8651_DEBUG
		printk("pen down\n");
#endif

#ifdef CONFIG_BATT_EXTERNAL

		ret = sx8651_get_resoulce(ts, SX8651_PEN_DOWN, &curst);	

		if(ret == 0)
		{
			if(curst == SX8651_BATTERY_CMD)
			{
				sx8651_get_resoulce(ts, SX8651_BATTERY_BAD, &curst);
				//goto sx8651_clear_irq_;
			}
			else
			{
#ifdef CONFIG_SX8651_DEBUG
				printk("%s,%d\n", __func__, curst);
#endif
			}	
		}
		else
		{
			ts->sx8651_delay_time = 0;
		}
#endif

		spin_lock_irqsave(&ts->lock, flags);
		disable_irq_nosync(ts->irq);
		ts->irq_st = TS_IRQ_DISABLE;
		/* the reading of the samples can be time-consuming if using
		 * a slow i2c, so the work is done in a queue */

		/*queue_work(ts->ts_workq, &ts->pen_event_work);*/
		queue_delayed_work(ts->ts_workq, &ts->pen_event_work, 0);
		spin_unlock_irqrestore(&ts->lock, flags);
	}

sx8651_clear_irq_:
	if (ts->clear_penirq)
		ts->clear_penirq();

#ifdef CONFIG_SX8651_DEBUG
	printk("return IRQ_HANDLE\n");
#endif
	return IRQ_HANDLED;
}

#ifdef CONFIG_BATT_EXTERNAL

static int sx8651_battery_check_end(struct sx8651 * ts)
{

	uint32_t ret = -1;

	if (ts)
	{
		/*remove calrunning*/
		ts->battery_flags &= ~BATTERY_FLAG_CHECKING;
		/* Enable Default Channels */
		if (sx865x_write_register(ts,I2C_REG_CHANMASK, CHAN_MASK))
		{
			/* Go into Automatic Mode */
      			if (sx865x_write_register(ts,I2C_REG_CTRL0,
                                    DEFAULT_RATE | DEFAULT_DELAY))
      			{ 
				ret = 0;
#ifdef CONFIG_SX8651_BATTERY_DEBUG 
	      			printk("func %s, battery check finish\n", __func__);
#endif
      			}
    		}
  	}
	return ret;
}

static void sx8651_battery_check_start(struct sx8651 * ts)
{
	uint32_t ret;
	uint32_t curst;

	if(ts)
	{
	/* Go into Manual Mode */
#ifdef CONFIG_SX8651_BATTERY_DEBUG 
	printk("%s , %d\n", __func__, __LINE__);	
#endif
	if (sx865x_write_command(ts,SX8651_POWDWN))
	{
#ifdef CONFIG_SX8651_BATTERY_DEBUG 
	printk("%s , %d\n", __func__, __LINE__);	
#endif
		ret = sx8651_get_resoulce(ts, SX8651_BATTERY_OK, &curst);
		if(ret == 0)
		{
#ifdef CONFIG_SX8651_BATTERY_DEBUG 
			printk("%s, %d, %d\n",__func__, __LINE__, curst);
#endif
			if(sx8651_battery_check_end(ts) == 0)
			{
				ret = sx8651_get_resoulce(ts, SX8651_IDLE, &curst);
				if(ret == 0)
				{
#ifdef CONFIG_SX8651_BATTERY_DEBUG 
					printk("%s, %d, %d\n",__func__, __LINE__, curst);
#endif
				}
			}
			return 0;
		}		
	
		/* Finish going into Manual Mode */
		if (sx865x_write_register(ts,I2C_REG_CTRL0,0))
		{
			/* Enable ONLY AUX */
			if (sx865x_write_register(ts,I2C_REG_CHANMASK,CONV_AUX))
			{
				/* Perform Select Command, this will in effect disable pen
				 	detection. */
				if (sx865x_write_command(ts,SX8651_SELECT_CH(CH_AUX)))
				{
					ts->battery_flags |= BATTERY_FLAG_CHECKING;
			
					/* Perform Convert Command */
					if (sx865x_write_command(ts,SX8651_CONVERT_CH(CH_SEQ)))
					{
						ret = sx8651_get_resoulce(ts, SX8651_BATTERY_DOWN, &curst);
						if(ret == 0)
						{
#ifdef CONFIG_SX8651_BATTERY_DEBUG 
							printk("%s, %d, %d\n",__func__, __LINE__, curst);
#endif
						}

#ifdef CONFIG_SX8651_BATTERY_DEBUG
						printk("func %s , battery command ok\n",__func__);
#endif				
					}	
				}
			}
		}
	}
	}

}

static void sx8651_battery_worker(struct work_struct *work)
{
	uint32_t ret;
	uint32_t curst;

#ifdef CONFIG_SX8651_BATTERY_DEBUG
	printk("func %s , line = %d\n",__func__, __LINE__);
#endif
	struct sx8651 *ts = container_of(work, struct sx8651, battery_event_work);

	ts->sx8651_delay_time++;	
	
	if(ts->sx8651_delay_time >= SX8651_BATTERY_TIME)
	{
		ret = sx8651_get_resoulce(ts, SX8651_BATTERY_CMD, &curst);
#ifdef CONFIG_SX8651_BATTERY_DEBUG
		printk("%s, %d\n", __func__, curst);
#endif
		if(ret == 1)
		{
			sx8651_battery_check_start(ts);	
		}
		else
		{
			if(curst != SX8651_PEN_DOWN)
			{
#ifdef CONFIG_SX8651_BATTERY_DEBUG
				printk("%s, %d\n", __func__, curst);
#endif
			}
		}
        }
        hrtimer_start(&ts->battery_timer, ktime_set(10, TS_POLL_PERIOD),\
                        HRTIMER_MODE_REL);
}


static enum hrtimer_restart imap_addc_timer(struct hrtimer *handle)
{
#ifdef CONFIG_SX8651_BATTERY_DEBUG
	printk("func %s , line = %d\n",__func__, __LINE__);
#endif
	struct sx8651 *ts = container_of(handle, struct sx8651, battery_timer);
	
	schedule_work(&ts->battery_event_work);	

	return HRTIMER_NORESTART;	
}
#endif

static int sx8651_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sx8651 *ts;
	struct sx865x_platform_data *pdata = pdata = client->dev.platform_data;
	struct input_dev *input_dev;
	int err;
	char buf1;

	dev_info(&client->dev, "sx8651_probe()\n");

	if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

	ts = kzalloc(sizeof(struct sx8651), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_ts;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts->input = input_dev;

	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = sx8651_timer_handler;

	spin_lock_init(&ts->lock);

	ts->pdata				= pdata;
	ts->model				= pdata->model;
	ts->y_plate_ohms		= pdata->y_plate_ohms;
	ts->get_pendown_state	= pdata->get_pendown_state;
	ts->clear_penirq		= pdata->clear_penirq;

	pdata->init_platform_hw();

	snprintf(ts->phys, sizeof(ts->phys),
		 "%s/input0", dev_name(&client->dev));

	input_dev->name = "SX8651 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_I2C;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	set_bit(EV_SYN, input_dev->evbit);
	
#ifdef CONFIG_SX8651_KEY
	home_key_st = 0;
	menu_key_st = 0;
	back_key_st = 0;
	search_key_st = 0;

	set_bit(KEY_HOME, input_dev->keybit);
	set_bit(KEY_SEARCH, input_dev->keybit);
	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_MENU, input_dev->keybit);

	file_read_st = 0;
	data_read_st = 0;
#endif
	/* register as multitouch device: 
      set ABS_MT_TOUCH_MAJOR, ABS_MT_POSITION_X, ABS_MT_POSITION_Y*/
#if 0
#ifdef TSLIB
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, MAX_12BIT, 0, 0);
#else
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, pdata->y_max, 0, 0);
#endif
#endif

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, MAX_12BIT, 0, 0);

	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, MAX_12BIT, 0, 0);

	err = input_register_device(input_dev);
	if (err)
		goto err_free_hrtimer;

	err = sysfs_create_group(&client->dev.kobj, &sx8651_attr_group);
	if (err)
		goto err_free_hrtimer;

	/****** hardware init.. **********/
	err = i2c_smbus_write_byte_data(client, I2C_REG_SOFTRESET, 
                                                        SOFTRESET_VALUE);
	/* soft reset: SX8651 does not nak at the end of a software reset, 
     ignore checking return value */
 
       	/* set mask to select channels to be converted */
	err = i2c_smbus_write_byte_data(client, I2C_REG_CHANMASK, CHAN_MASK);
	if (err != 0) {
		dev_err(&client->dev, "write mask fail");
		goto err_remove_attr_group_;
	}

	err = i2c_smbus_write_byte_data(client, I2C_REG_CTRL1, 
                                  CONDIRQ | FILT_7SA ); 

	if (err != 0) {
		dev_err(&client->dev, "writereg1 fail");
		goto err_remove_attr_group_;
	}

	/* POWDLY/SETDLY settling time -- adjust TS_TIMEOUT accordingly */
	err = i2c_smbus_write_byte_data(client, I2C_REG_CTRL0, DEFAULT_DELAY); // DLY_SINGLE_TOUCH);	// PWDLY
	if (err != 0) {
		dev_err(&client->dev, "writereg0 fail");
		goto err_remove_attr_group_;
	}
	err = i2c_smbus_write_byte_data(client, I2C_REG_CTRL2, DEFAULT_DELAY); // DLY_1_1US);	// SETDLY
	if (err != 0) {
		dev_err(&client->dev, "writereg0 fail");
		goto err_remove_attr_group_;
	}

	ts->ts_workq = create_singlethread_workqueue("sx8651");
	if (ts->ts_workq == NULL) {
		dev_err(&client->dev, "failed to create workqueue\n");
		goto err_remove_attr_group_;
	}

	//INIT_WORK(&ts->pen_event_work, sx8651_pen_irq_worker);
	INIT_DELAYED_WORK(&ts->pen_event_work, sx8651_pen_irq_worker);


	ts->irq_st = TS_IRQ_ENABLE;

	client->irq = irq_no;
	ts->irq = client->irq;

	//err = request_irq(ts->irq, sx8651_irq, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_DISABLED,
	//		client->dev.driver->name, ts);

	err = request_irq(ts->irq, sx8651_irq, IRQF_TRIGGER_LOW | IRQF_DISABLED,
                client->dev.driver->name, ts);

	if (err < 0) {
		dev_err(&client->dev, "irq %d busy?\n", ts->irq);
		goto err_remove_attr_group_;
	}

	dev_info(&client->dev, "registered with irq (%d)\n", ts->irq);

	/* enter pen-trigger mode */
	err = i2c_smbus_write_byte(client, SX8651_PENTRG);
	if (err != 0) {
		dev_err(&client->dev, "enter fail");
		goto err_free_irq_;
	}

	ts->width_threshold = _INITIAL_OMAP3EVM_WIDTH_THRESHOLD;
	ts->height_threshold = _INITIAL_OMAP3EVM_HEIGHT_THRESHOLD;
	ts->_ns_count = 0;

#ifdef CONFIG_BATT_EXTERNAL
#ifdef CONFIG_SX8651_BATTERY_DEBUG
	printk("func %s line = %d\n",__func__, __LINE__);
#endif
	spin_lock_init(&ts->sx8651_lock);
	ts->sx8651_manual = SX8651_IDLE;

	ts->sx8651_delay_time = SX8651_BATTERY_TIME;

	ts->battery_flags &= ~BATTERY_FLAG_CHECKING;	

	INIT_WORK(&ts->battery_event_work, sx8651_battery_worker);

        hrtimer_init(&ts->battery_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        ts->battery_timer.function = imap_addc_timer;
	
//	if(batt_init_ok ==1)
//		supply_timer_func();

#endif

	sx8651_StartNoTouchCalibration(ts);

	printk("EINTCON = 0x%x\n", __raw_readl(rEINTCON));
#ifdef CONFIG_BATT_EXTERNAL
        hrtimer_start(&ts->battery_timer, ktime_set(0, TS_POLL_PERIOD),\
                        HRTIMER_MODE_REL);
#endif
	return 0;

 err_free_irq_:
	free_irq(ts->irq, ts);
 err_remove_attr_group_:
	sysfs_remove_group(&client->dev.kobj, &sx8651_attr_group);
	input_unregister_device(input_dev);
 err_free_hrtimer:
	hrtimer_cancel(&ts->timer);
 err_free_ts:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}

static int sx8651_remove(struct i2c_client *client)
{
	struct sx8651  *ts = i2c_get_clientdata(client);
	struct sx865x_platform_data *pdata;

	pdata = client->dev.platform_data;
	pdata->exit_platform_hw();

//	cancel_work_sync(&ts->pen_event_work);
	cancel_delayed_work_sync(&ts->pen_event_work);
	destroy_workqueue(ts->ts_workq);

	sysfs_remove_group(&ts->client->dev.kobj, &sx8651_attr_group);

	free_irq(ts->irq, ts);
	hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input);
	kfree(ts);

	return 0;
}

static int sx8651_suspend(struct i2c_client *client)
{
	return 0;
}

static int sx8651_resume(struct i2c_client *client)
{
	uint32_t err = 0;

	struct sx8651  *ts = i2c_get_clientdata(client);

	 printk("func = %s start\n", __func__);

	 imapx_gpio_setcfg(sx8651_int, IG_INPUT, IG_NORMAL);
	 imapx_gpio_setirq(sx8651_int, FILTER_MAX, IG_LOW, 1);
/*
	volatile unsigned int tmp = 0;
        tmp = __raw_readl(rEINTCON);
        tmp &= ~(7 << 4);
        tmp |=(7 << 4);
        __raw_writel(tmp , rEINTCON);
        tmp = __raw_readl(rEINTFLTCON0);
        tmp &= ~(0xff << 8);
        tmp |= (0x1 << 15) | (0x3f << 8);
        __raw_writel(tmp, rEINTFLTCON0);
*/
	 udelay(1000);

         /****** hardware init.. **********/
         err = i2c_smbus_write_byte_data(client, I2C_REG_SOFTRESET,
                                                         SOFTRESET_VALUE);
         /* soft reset: SX8651 does not nak at the end of a software reset, 
 	       ignore checking return value */
 
         /* set mask to select channels to be converted */
         err = i2c_smbus_write_byte_data(client, I2C_REG_CHANMASK, CHAN_MASK);
         if (err != 0) {
                 dev_err(&client->dev, "write mask fail");
                 goto sx8651_err_resume_;
         }
 
         err = i2c_smbus_write_byte_data(client, I2C_REG_CTRL1,
                                   CONDIRQ | FILT_7SA );
 
         if (err != 0) {
                 dev_err(&client->dev, "writereg1 fail");
                 goto sx8651_err_resume_;
         }
 
         /* POWDLY/SETDLY settling time -- adjust TS_TIMEOUT accordingly */
         err = i2c_smbus_write_byte_data(client, I2C_REG_CTRL0, DEFAULT_DELAY); // DLY_SINGLE_TOUCH);    // PWDLY
         if (err != 0) {
                 dev_err(&client->dev, "writereg0 fail");
                 goto sx8651_err_resume_;
         }
         err = i2c_smbus_write_byte_data(client, I2C_REG_CTRL2, DEFAULT_DELAY); // DLY_1_1US);   // SETDLY
         if (err != 0) {
                 dev_err(&client->dev, "writereg0 fail");
                 goto sx8651_err_resume_;
         }

       /* enter pen-trigger mode */
       err = i2c_smbus_write_byte(client, SX8651_PENTRG);
       if (err != 0) {
               dev_err(&client->dev, "enter fail");
               goto sx8651_err_resume_;
       }
	ts->width_threshold = _INITIAL_OMAP3EVM_WIDTH_THRESHOLD;
	ts->height_threshold = _INITIAL_OMAP3EVM_HEIGHT_THRESHOLD;
	ts->_ns_count = 0;

	sx8651_StartNoTouchCalibration(ts);

	 printk("func = %s end\n", __func__);
	return 0;

sx8651_err_resume_:

	return -1;

}

static struct i2c_device_id sx8651_idtable[] = {
	{ "sx8651", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, sx8651_idtable);

static struct i2c_driver sx8651_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = "sx8651"
	},
	.id_table       = sx8651_idtable,
	.probe	  = sx8651_probe,
	.remove	 = sx8651_remove,
	.suspend        = sx8651_suspend,
	.resume         = sx8651_resume,

};

static int sx_get_pendown_state(void)
{
	return !(imapx_gpio_getpin(sx8651_int, IG_NORMAL));      /* Touchscreen PENIRQ */
}

static void sx_clear_penirq(void)
{
//	printk("EINTCON = %x\n", __raw_readl(rEINTCON));

	//__raw_writel(0x2, rSRCPND);
	//__raw_writel(0x2, rINTPND);
}

static int sx_init_ts(void)
{
	/* was already done in sx8651_dev_init() */

	return 0;
}

static void sx_exit_ts(void)
{
}

static struct sx865x_platform_data sx8651_data = {
	.x_max                  = 480,  // screen resolution
	.y_max                  = 640,  // screen resolution
	.model = 8651,
	.y_plate_ohms = 250,
	.get_pendown_state = sx_get_pendown_state,
	.clear_penirq      = sx_clear_penirq,
	.init_platform_hw  = sx_init_ts,
	.exit_platform_hw  = sx_exit_ts,
};

static struct i2c_board_info sx8651_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("sx8651", 0x48),
		.flags = I2C_CLIENT_WAKE,
		//.irq   = IRQ_EINT1,
		.platform_data = &sx8651_data,
	},
};

static int imap_i2c_init_sx8651(void)
{
	struct i2c_adapter *adp;
//	i2c_register_board_info(1, sx8651_i2c_boardinfo, ARRAY_SIZE(sx8651_i2c_boardinfo));

	adp = i2c_get_adapter(CONFIG_TP_SX8651_I2C + 1);
	i2c_new_device(adp, sx8651_i2c_boardinfo);
	return 0;
}

static void sx8651_dev_init(void)
{
	sx8651_int  = __imapx_name_to_gpio(CONFIG_TP_SX8651_INT);
	if(sx8651_int == IMAPX_GPIO_ERROR) {
		printk(KERN_ERR "failed to get sx8651_int pin.\n");
		return -1;
	}   
	irq_no = imapx_gpio_to_irq(sx8651_int);
	imapx_gpio_setcfg(sx8651_int, IG_INPUT, IG_NORMAL);
	imapx_gpio_setirq(sx8651_int, FILTER_MAX, IG_LOW, 1);
/*
	volatile unsigned int tmp = 0;
	tmp = __raw_readl(rEINTCON);
	tmp &= ~(7 << 4);
	//	tmp |=(7 << 4);
	__raw_writel(tmp , rEINTCON);
	tmp = __raw_readl(rEINTFLTCON0);
	tmp &= ~(0xff << 8);
	tmp |= (0x1 << 15) | (0x3f << 8);
	__raw_writel(tmp, rEINTFLTCON0);
*/
	udelay(1000);
}

static int __init sx8651_init(void)
{
#ifdef CONFIG_BATT_EXTERNAL
	__imapx_register_batt(sx8651_get_adc_val);
#endif
	sx8651_dev_init();
	imap_i2c_init_sx8651();
	return i2c_add_driver(&sx8651_driver);
}

static void __exit sx8651_exit(void)
{
	i2c_del_driver(&sx8651_driver);
}

module_init(sx8651_init);
module_exit(sx8651_exit);

MODULE_AUTHOR("Semtech Corp.");
MODULE_DESCRIPTION("SX8651 TouchScreen Driver");
MODULE_LICENSE("GPL");

