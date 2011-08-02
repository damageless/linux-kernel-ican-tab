
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

#if 1
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


struct sx8651_lite {
	char			phys[32];
	struct hrtimer		battery_timer;
	struct ts_event		tc;

	struct i2c_client       *client;

	struct sx865x_platform_data	*pdata;

	struct work_struct	battery_event_work;
	char			battery_flags;
	spinlock_t		lock;

	u16			model;

	char			debug;
	char			bytesToRead;

	char			reg0Value; /* memory value of reg0 */

	int32_t			sx8651_delay_time;
	spinlock_t		sx8651_lock;	
};


#define SX8651_BATTERY_TIME  (10)

int battery_val=1172;
int sx8651_get_adc_val(void)
{
	return battery_val;
}

/*
 * Function Name: sx865x_write_command()
 * Purpose: Easy access for writting commands to SX865X
 * Arguments: struct sx8651 * - pointer to struct data
 *            unsigned char - command to write
 * Returns: 1 - if successful, otherwise 0
 */
static char sx865x_write_command(struct sx8651_lite *this, 
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
static char sx865x_write_register(struct sx8651_lite *this,
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
static int sx865x_read_register(struct sx8651_lite *ts, unsigned char reg_addr)
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


/*
 * Function Name: sx8651_read_values()
 * Purpose: Reads touch screen channel data from the device.
 * Description: This assumes that an interrupt was made.
 * Arguments: struct sx8651 * - pointer to struct data
 * Returns: 0 - successful, -1 - unsuccessful
 */
static int sx8651_read_values(struct sx8651_lite *tsc)
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
				battery_val = tsc->tc.aux;
//				printk("battery_val = %x\n", battery_val);
			} else {
				printk(KERN_ERR "? %d %x\n", ch, data & 0xfff);
				ret = -1;
			}

		}
	} else {
		printk(KERN_ERR "%d = recv()\n", length);
		//	ret = -1;
	}
	return ret;
}


static void sx8651_battery_worker(struct work_struct *work)
{
	uint32_t ret;

	struct sx8651_lite *ts = container_of(work, struct sx8651_lite, battery_event_work);

	ts->sx8651_delay_time++;	

	if(ts->sx8651_delay_time >= SX8651_BATTERY_TIME)
	{
		if (sx865x_write_command(ts,SX8651_POWDWN))
		{
			if (sx865x_write_register(ts,I2C_REG_CTRL0,0))
			{
				if (sx865x_write_register(ts,I2C_REG_CHANMASK,CONV_AUX))
				{
					if (sx865x_write_command(ts, SX8651_SELECT_CH(CH_AUX)))
					{
						ts->battery_flags |= BATTERY_FLAG_CHECKING;
						if (sx865x_write_command(ts,SX8651_CONVERT_CH(CH_AUX)))
						{
							ret = sx8651_read_values(ts);
						}
					}
				}
			}
		}
	}
	hrtimer_start(&ts->battery_timer, ktime_set(10, TS_POLL_PERIOD),\
			HRTIMER_MODE_REL);
}


static enum hrtimer_restart imap_addc_timer(struct hrtimer *handle)
{
	struct sx8651_lite *ts = container_of(handle, struct sx8651_lite, battery_timer);

	schedule_work(&ts->battery_event_work);	

	return HRTIMER_NORESTART;	
}

static int sx8651_lite_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct sx8651_lite *ts;
	struct sx865x_platform_data *pdata = pdata = client->dev.platform_data;
	int err;
	char buf1;

	dev_info(&client->dev, "sx8651_lite_probe()\n");

	if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

	ts = kzalloc(sizeof(struct sx8651_lite), GFP_KERNEL);
	if (!ts) {
		err = -ENOMEM;
		goto err_free_ts;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);


	ts->pdata		= pdata;
	ts->model		= pdata->model;

	pdata->init_platform_hw();

	snprintf(ts->phys, sizeof(ts->phys),
			"%s/input0", dev_name(&client->dev));


	/****** hardware init.. **********/
	err = i2c_smbus_write_byte_data(client, I2C_REG_SOFTRESET, 
			SOFTRESET_VALUE);
	/* soft reset: SX8651 does not nak at the end of a software reset, 
	   ignore checking return value */

	/* set mask to select channels to be converted */
	err = i2c_smbus_write_byte_data(client, I2C_REG_CHANMASK, CHAN_MASK);
	if (err != 0) {
		dev_err(&client->dev, "write mask fail");
		goto err_free_ts;
	}

	err = i2c_smbus_write_byte_data(client, I2C_REG_CTRL1, 
			CONDIRQ | FILT_7SA ); 

	if (err != 0) {
		dev_err(&client->dev, "writereg1 fail");
		goto err_free_ts;
	}

	/* POWDLY/SETDLY settling time -- adjust TS_TIMEOUT accordingly */
	err = i2c_smbus_write_byte_data(client, I2C_REG_CTRL0, DEFAULT_DELAY); // DLY_SINGLE_TOUCH);	// PWDLY
	if (err != 0) {
		dev_err(&client->dev, "writereg0 fail");
		goto err_free_ts;
	}
	err = i2c_smbus_write_byte_data(client, I2C_REG_CTRL2, DEFAULT_DELAY); // DLY_1_1US);	// SETDLY
	if (err != 0) {
		dev_err(&client->dev, "writereg2 fail");
		goto err_free_ts;
	}

	printk("func %s line = %d\n",__func__, __LINE__);
	spin_lock_init(&ts->sx8651_lock);

	ts->sx8651_delay_time = SX8651_BATTERY_TIME;

	ts->battery_flags &= ~BATTERY_FLAG_CHECKING;	

	INIT_WORK(&ts->battery_event_work, sx8651_battery_worker);

	hrtimer_init(&ts->battery_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->battery_timer.function = imap_addc_timer;



	hrtimer_start(&ts->battery_timer, ktime_set(0, TS_POLL_PERIOD),\
			HRTIMER_MODE_REL);
	return 0;

err_free_ts:
	kfree(ts);
	return err;
}

static int sx8651_lite_remove(struct i2c_client *client)
{
	struct sx8651_lite  *ts = i2c_get_clientdata(client);
	struct sx865x_platform_data *pdata;

	pdata = client->dev.platform_data;
	pdata->exit_platform_hw();

	kfree(ts);

	return 0;
}

static int sx8651_lite_suspend(struct i2c_client *client)
{
	return 0;
}

static int sx8651_lite_resume(struct i2c_client *client)
{
	uint32_t err = 0;

	struct sx8651_lite  *ts = i2c_get_clientdata(client);

	printk("func = %s start\n", __func__);

	volatile unsigned int tmp = 0;
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

	printk("func = %s end\n", __func__);
	return 0;

sx8651_err_resume_:

	return -1;

}

static struct i2c_device_id sx8651_lite_idtable[] = {
	{ "sx8651_lite", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, sx8651_lite_idtable);

static struct i2c_driver sx8651_lite_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = "sx8651_lite"
	},
	.id_table       = sx8651_lite_idtable,
	.probe		= sx8651_lite_probe,
	.remove		= sx8651_lite_remove,
	.suspend        = sx8651_lite_suspend,
	.resume         = sx8651_lite_resume,

};

static int sx_get_pendown_state(void)
{
	return 0;
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


static struct i2c_board_info sx8651_lite_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("sx8651_lite", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq   = IRQ_EINT1,
		.platform_data = &sx8651_data,
	},
};

static int imap_i2c_init_sx8651_lite(void)
{
	struct i2c_adapter *adp;

	//adp = i2c_get_adapter(1);
	adp = i2c_get_adapter(CONFIG_TP_SX8651_I2C + 1);
	i2c_new_device(adp, sx8651_lite_i2c_boardinfo);
	return 0;
}

static void sx8651_lite_dev_init(void)
{
	udelay(1000);
}

static int __init sx8651_lite_init(void)
{
	__imapx_register_batt(sx8651_get_adc_val);
	sx8651_lite_dev_init();
	imap_i2c_init_sx8651_lite();
	return i2c_add_driver(&sx8651_lite_driver);
}

static void __exit sx8651_lite_exit(void)
{
	i2c_del_driver(&sx8651_lite_driver);
}

module_init(sx8651_lite_init);
module_exit(sx8651_lite_exit);

MODULE_AUTHOR("Semtech Corp.");
MODULE_DESCRIPTION("SX8651 TouchScreen Driver");
MODULE_LICENSE("GPL");
#endif

