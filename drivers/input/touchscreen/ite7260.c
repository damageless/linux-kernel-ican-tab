/* drivers/input/touchscreen/IT7260_ts_i2c.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <asm/uaccess.h>
#include <linux/input.h>
#include <asm/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include "IT7260_ts.h"

#define IT7260_I2C_NAME "IT7260"
#include <linux/gpio.h>
static int ite7260_major = 0; // dynamic major by default
static int ite7260_minor = 0;
static struct cdev ite7260_cdev;
static struct class *ite7260_class = NULL;
static dev_t ite7260_dev;
static struct input_dev *input_dev;
static struct timer_list irq_timer;
int g_tpirq_count = 0;
int g_ts_enabled = 0;
EXPORT_SYMBOL(g_ts_enabled);

#ifdef CONFIG_HDMI_OUTPUT_SUPPORT
extern struct completion Menu_Button;
#endif
extern int imap_iokey_led(int, int);
extern int imap_iokey_motor(int, int);

unsigned int ite7260_vcc;
unsigned int ite7260_int;
unsigned int ite7260_rst;
//#define DEBUG

#ifdef DEBUG
#define TS_DEBUG(fmt,args...)  printk( KERN_ERR "[it7260_i2c]: " fmt, ## args)
#define DBG() printk("[%s]:%d => \n",__FUNCTION__,__LINE__)
#else
#define TS_DEBUG(fmt,args...)
#define DBG()
#endif

struct workqueue_struct *IT7260_wq;

struct IT7260_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct delayed_work work;
	struct early_suspend early_suspend;
	uint8_t debug_log_level;
};
#ifdef CONFIG_HAS_EARLYSUSPEND
static void IT7260_ts_early_suspend(struct early_suspend *h);
static void IT7260_ts_late_resume(struct early_suspend *h);
#endif

struct IT7260_ts_data gltso, *gl_ts;

int i2cReadFromIt7260(struct i2c_client *client, unsigned char bufferIndex,
		unsigned char dataBuffer[], unsigned short dataLength) {
	int ret;
	struct i2c_msg msgs[2] = { 
		{
		       	.addr	= client->addr,
		       	.flags	= I2C_M_NOSTART,
			.len	= 1,
		       	.buf	= &bufferIndex
		},{ 
			.addr	= client->addr,
		       	.flags	= I2C_M_RD,
		       	.len	= dataLength,
		       	.buf	= dataBuffer
	       	}
       	};

	memset(dataBuffer, 0xFF, dataLength);
	ret = i2c_transfer(client->adapter, msgs, 2);
	return ret;
}

int i2cWriteToIt7260(struct i2c_client *client, unsigned char bufferIndex,
		unsigned char const dataBuffer[], unsigned short dataLength) {
	unsigned char buffer4Write[256];
	struct i2c_msg msgs[1] = { { .addr = client->addr, .flags = 0, .len =
			dataLength + 1, .buf = buffer4Write } };

	buffer4Write[0] = bufferIndex;
	memcpy(&(buffer4Write[1]), dataBuffer, dataLength);
	return i2c_transfer(client->adapter, msgs, 1);
}

static void Read_Point(struct IT7260_ts_data *ts) {
	unsigned char ucQuery = 0;
	unsigned char pucPoint[14];
#ifdef HAS_8_BYTES_LIMIT
	unsigned char cPoint[8];
	unsigned char ePoint[6];
#endif //HAS_8_BYTES_LIMIT
	int ret = 0;
	int finger2_pressed = 0;
	int xraw, yraw, xtmp, ytmp;
	int i = 0;
	static int x[2] = { (int) -1, (int) -1 };
	static int y[2] = { (int) -1, (int) -1 };
	static bool finger[2] = { 0, 0 };
	static bool key[4] = { 0, 0, 0, 0 };
	static uint32_t key_code[4] = { KEY_HOME, KEY_MENU, KEY_BACK, KEY_SEARCH };
	static bool flag = 0;

//	TS_DEBUG("in %s\n", __func__);
	i2cReadFromIt7260(ts->client, 0x80, &ucQuery, 1);
	if (ucQuery < 0) {
		TS_DEBUG("=error Read_Point=\n");
//		if (ts->use_irq)
//			enable_irq(ts->client->irq);
		return;
	} else {
		if (ucQuery & 0x80) {
#ifdef HAS_8_BYTES_LIMIT
			i2cReadFromIt7260(ts->client, 0xC0, cPoint, 8);
			ret = i2cReadFromIt7260(ts->client, 0xE0, ePoint, 6);
			for(i=0; i<6; i++) {
				pucPoint[i] = ePoint[i];
			}
			for(i=0; i<8; i++) {
				pucPoint[i+6] = cPoint[i];
			}
#else //HAS_8_BYTES_LIMIT
			ret = i2cReadFromIt7260(ts->client, 0xE0, pucPoint, 14);
#endif //HAS_8_BYTES_LIMIT
			//pr_info("=Read_Point read ret[%d]--point[%x][%x][%x][%x][%x][%x][%x][%x][%x][%x][%x][%x][%x][%x]=\n",
			//	ret,pucPoint[0],pucPoint[1],pucPoint[2],
			//	pucPoint[3],pucPoint[4],pucPoint[5],pucPoint[6],pucPoint[7],pucPoint[8],
			//	pucPoint[9],pucPoint[10],pucPoint[11],pucPoint[12],pucPoint[13]);
			if (ret) {
//				TS_DEBUG("pub0=%x, pub1=%x\n", pucPoint[0], pucPoint[1]);
				// gesture
				if (pucPoint[0] & 0xF0) {
//					if (ts->use_irq)
//						enable_irq(ts->client->irq);
					//pr_info("(pucPoint[0] & 0xF0) is true, it's a gesture\n") ;
					//pr_info("pucPoint[0]=%x\n", pucPoint[0]);
					if(pucPoint[0] == 0x41)
					{
						uint8_t key_num = pucPoint[1] - 1;
						if(key_num > 3)
						  printk(KERN_ERR "Unrecognized key_num %d received.\n",
							 key_num);
						else {
							if(!key[key_num]) {
								TS_DEBUG("Key %d pressed.\n", key_num);
								input_event(ts->input_dev,
								   EV_KEY, key_code[key_num], 1);
								key[key_num] = 1;
								imap_iokey_led(1, 10000);
								imap_iokey_motor(1, 40);
							}
						}
					}
					return;
				}
				// palm
				if (pucPoint[1] & 0x01) {
//					if (ts->use_irq)
//						enable_irq(ts->client->irq);
					TS_DEBUG("pucPoint 1 is 0x01, it's a palm\n") ;
					return;
				}
				// no more data
				if (!(pucPoint[0] & 0x08)) {
					if (finger[0]) {
						input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 1);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
						//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, pressure_point);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x[0]);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y[0]);
						input_mt_sync(ts->input_dev);
						finger[0] = 0;
						flag = 1;
						TS_DEBUG("finger 0 done, x=%d, y=%d\n", x[0], y[0]);
					}
					if (finger[1]) {
						input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 2);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
						//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, pressure_point);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x[1]);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y[1]);
						input_mt_sync(ts->input_dev);
						finger[1] = 0;
						flag = 1;
						TS_DEBUG("finger 1 done, x=%d, y=%d\n", x[1], y[1]);
					}

					for(i = 0; i < 4; i++)
					{
						if(key[i])
						{
							key[i] = 0;
							input_event(ts->input_dev, EV_KEY, key_code[i], 0);
							TS_DEBUG("key %d up.\n", i);
#ifdef CONFIG_HDMI_OUTPUT_SUPPORT
							if(key_code[i] == KEY_MENU)
							  complete(&Menu_Button);
#endif
						}
					}
					if (flag) {
						input_sync(ts->input_dev);
						flag = 0;
					}
//					if (ts->use_irq)
//						enable_irq(ts->client->irq);
					//pr_info("(pucPoint[0] & 0x08) is false, means no more data\n") ;
					return;
				}
				// 3 fingers
				if (pucPoint[0] & 0x04) {
//					if (ts->use_irq)
//						enable_irq(ts->client->irq);
					TS_DEBUG("(pucPoint[0] & 0x04) is true, we don't support three fingers\n") ;
					return;
				}

				if (pucPoint[0] & 0x01) {
					char pressure_point, z, w;

					xraw = ((pucPoint[3] & 0x0F) << 8) + pucPoint[2];
					yraw = ((pucPoint[3] & 0xF0) << 4) + pucPoint[4];

					pressure_point = pucPoint[5] & 0x0f;
					//pr_info("=Read_Point1 x=%d y=%d p=%d=\n",xraw,yraw,pressure_point);

					input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 1);
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 1);
					//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, pressure_point);
					input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, xraw);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, yraw);
					input_mt_sync(ts->input_dev);
					x[0] = xraw;
					y[0] = yraw;
					finger[0] = 1;
					TS_DEBUG("=input Read_Point1 x=%d y=%d p=%d=\n",xraw,yraw,pressure_point);
				} else if(finger[0]){
					input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 1);
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
					//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, pressure_point);
					input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x[0]);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y[0]);
					input_mt_sync(ts->input_dev);
					finger[0] = 0;
					TS_DEBUG("finger 0 up, x=%d y=%d\n",x[0],y[0]);
				}

				if (pucPoint[0] & 0x02) {
					char pressure_point, z, w;
					xraw = ((pucPoint[7] & 0x0F) << 8) + pucPoint[6];
					yraw = ((pucPoint[7] & 0xF0) << 4) + pucPoint[8];

					pressure_point = pucPoint[9] & 0x0f;

					//pr_info("=Read_Point2 x=%d y=%d p=%d=\n",xraw,yraw,pressure_point);
					input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 2);
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 1);
					//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, pressure_point);
					input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, xraw);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, yraw);
					input_mt_sync(ts->input_dev);
					x[1] = xraw;
					y[1] = yraw;
					finger[1] = 1;
					TS_DEBUG("input Read_Point2 x=%d y=%d p=%d=\n",xraw,yraw,pressure_point);
				} else if (finger[1]){
					input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 2);
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
					//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, pressure_point);
					input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x[1]);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y[1]);
					input_mt_sync(ts->input_dev);
					finger[1] = 0;
					TS_DEBUG("finger 1 up, x=%d y=%d\n",x[1],y[1]);
				}
				input_sync(ts->input_dev);
			}
		}
#if 0
		else
		  TS_DEBUG("No infomation detected. rubbish interrupt.\n");
#endif
	}
//	if (ts->use_irq)
//		enable_irq(ts->client->irq);
	//pr_info("=end Read_Point=\n");

}

void sendIdleCmd(struct IT7260_ts_data *ts) {
	int ret = 0;
	uint8_t cmd[] = {0x4, 0, 1};

	ret = i2cWriteToIt7260(ts->client, 0x20, cmd, 3);
	TS_DEBUG("sent idle cmd [%d]!!!\n", ret);
}

static int it7260_hwinit(void)
{
	int ret = 0;
	int irq = 0;
	int err = 0;
	volatile int tmp = 0;

	// RESET HIGH
	imapx_gpio_setcfg(ite7260_rst, IG_OUTPUT, IG_NORMAL);
	imapx_gpio_setpin(ite7260_rst, 1, IG_NORMAL);
	// POWER DOWN
	imapx_gpio_setcfg(ite7260_vcc, IG_OUTPUT, IG_NORMAL);
	imapx_gpio_setpin(ite7260_vcc, 0, IG_NORMAL);
	// INT LOW
	imapx_gpio_setcfg(ite7260_int, IG_OUTPUT, IG_NORMAL);
	imapx_gpio_setpin(ite7260_int, 0, IG_NORMAL);
	mdelay(100);
	// POWER UP
	imapx_gpio_setpin(ite7260_vcc, 1, IG_NORMAL);
	mdelay(50);
	// INT HIGH
	imapx_gpio_setpin(ite7260_int, 1, IG_NORMAL);
	mdelay(100);
	// INT INPUT
	imapx_gpio_setcfg(ite7260_int, IG_INPUT, IG_NORMAL);
	imapx_gpio_setirq(ite7260_int, FILTER_MAX, IG_BOTH, 1);

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////
void IT7260_ts_timer_func(unsigned long x)
{
	struct IT7260_ts_data *ts = gl_ts;

	if (ts->use_irq)
	  enable_irq(ts->client->irq);
}

static void IT7260_ts_work_func(struct work_struct *work) {
	int i;
	int ret;
	int bad_data = 0;
	struct i2c_msg msg[2];
	uint8_t start_reg;
	uint8_t buf[15];
//	TS_DEBUG("=IT7260_ts_work_func=\n"); 
	struct IT7260_ts_data *ts = container_of(work, struct IT7260_ts_data, work);
//	gl_ts = ts;
	Read_Point(ts);

	/* wait 20ms before allowing next press */
//	msleep(10);
//	sendIdleCmd(ts);
	mod_timer(&irq_timer, jiffies + msecs_to_jiffies(20));
}

static irqreturn_t IT7260_ts_irq_handler(int irq, void *dev_id) {
	struct IT7260_ts_data *ts = gl_ts;
	uint32_t pnd;

//	if(!((++g_tpirq_count) & 0xff))
//		printk(KERN_ERR "g_tpirq_count = %d\n", g_tpirq_count);
//	TS_DEBUG("ts_irq ++\n");
	pnd = readl(rEINTG4PEND);
	if(!(pnd & (1 << 3)))      
	  return IRQ_HANDLED;

	/* mask the interrupt temperory out */      
	writel(readl(rEINTG4MASK) | (1 << 3), rEINTG4MASK);

	/* clear pend */       
	writel((1 << 3), rEINTG4PEND);

//	TS_DEBUG("=IT7260_ts_irq_handler=\n");
	disable_irq_nosync(ts->client->irq);

//	TS_DEBUG("ts_irq ++, wq=%p, work=%p\n", IT7260_wq, &ts->work);
//	queue_delayed_work(IT7260_wq, &ts->work, 0);
//	schedule_delayed_work(&ts->work, 3);
	schedule_work(&ts->work);
//	TS_DEBUG("work done\n");

	/* unmask the interrupt temperory out */
	writel(readl(rEINTG4MASK) & ~(1 << 3), rEINTG4MASK);
	return IRQ_HANDLED;
}
/////////////////////////////////////////////////////////
void sendCalibrationCmd(struct IT7260_ts_data *ts)
{
	int ret = 0;
	unsigned char data[] = { 0x13, 0x00, 0x00, 0x00, 0x00 };
	unsigned char resp[2];

	ret = i2cWriteToIt7260(ts->client, 0x20, data, 5);
	printk(KERN_INFO "IT7260 sent calibration command [%d]!!!\n", ret);

	//TODO:
	//MUST sleep 5 seconds here!
	msleep(900);
	msleep(900);
	msleep(900);
	msleep(900);
	msleep(900);
	msleep(900);
	msleep(900);
	msleep(900);
	msleep(900);
	msleep(900);

	//Read out response to clear interrupt.
	i2cReadFromIt7260(ts->client, 0xA0, resp, 2);
	TS_DEBUG("got calibrate responce [%x]\n", *(uint16_t *)resp);
}
EXPORT_SYMBOL( sendCalibrationCmd);

void sendTimerCmd(struct IT7260_ts_data *ts) {
	int ret = 0;
	unsigned char data1[] = { 0x12, 0x01, 0x00, 0x02, 0x00, 0x00};
	unsigned char data2[] = { 0x11, 0x01, 0x00};
	unsigned char data3[] = { 0x12, 0x00, 0x20, 0x00, 0x00, 0x00};
	unsigned char data4[] = { 0x11, 0x00, 0x01};
	unsigned char resp[2];

	ret = i2cWriteToIt7260(ts->client, 0x20, data1, 6);
	TS_DEBUG("sent sleep cmd [%d]!!!\n", ret);

	i2cReadFromIt7260(ts->client, 0xa0, resp, 2);
	TS_DEBUG("got sleep responce [%x]\n", *(uint16_t *)resp);

	ret = i2cWriteToIt7260(ts->client, 0x20, data3, 6);
	TS_DEBUG("sent sleep cmd [%d]!!!\n", ret);

	i2cReadFromIt7260(ts->client, 0xa0, resp, 2);
	TS_DEBUG("got sleep responce [%x]\n", *(uint16_t *)resp);
#if 1
	ret = i2cWriteToIt7260(ts->client, 0x20, data2, 3);
	TS_DEBUG("sent sleep enable cmd [%d]!!!\n", ret);

	i2cReadFromIt7260(ts->client, 0xa0, resp, 2);
	TS_DEBUG("got sleep responce [%x]\n", *(uint16_t *)resp);

	ret = i2cWriteToIt7260(ts->client, 0x20, data4, 3);
	TS_DEBUG("sent sleep enable cmd [%d]!!!\n", ret);

	i2cReadFromIt7260(ts->client, 0xa0, resp, 2);
	TS_DEBUG("got sleep responce [%x]\n", *(uint16_t *)resp);
#endif
}

int IT7260_dev_create(void);

static int IT7260_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *id) {
	struct IT7260_ts_data *ts;
	struct i2c_msg msg[2];
	int ret = 0;
	struct IT7260_i2c_platform_data *pdata;
	unsigned long irqflags;
	unsigned char ucQuery = 0;
	irqflags = IRQF_TRIGGER_HIGH;
	pr_info("=entry IT7260_ts_probe=\n");

	ite7260_vcc  = __imapx_name_to_gpio(CONFIG_TP_ITE7260_POWER);
	if(ite7260_vcc == IMAPX_GPIO_ERROR) {
		printk(KERN_ERR "failed to get ite7260_vcc pin.\n");
		return -1;
	}
	ite7260_int  = __imapx_name_to_gpio(CONFIG_TP_ITE7260_INT);
	if(ite7260_int == IMAPX_GPIO_ERROR) {
		printk(KERN_ERR "failed to get ite7260_int pin.\n");
		return -1;
	}
	ite7260_rst  = __imapx_name_to_gpio(CONFIG_TP_ITE7260_RESET);
	if(ite7260_rst == IMAPX_GPIO_ERROR) {
		printk(KERN_ERR "failed to get ite7260_rst pin.\n");
		return -1;
	}

	it7260_hwinit();

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "IT7260_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_check_functionality_failed;
	}

	gl_ts = ts;
	ts->client = client;

	ts->debug_log_level = 0x3;

	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;

	IT7260_dev_create();
	g_ts_enabled = 1;

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	ts->early_suspend.suspend = IT7260_ts_early_suspend;
	ts->early_suspend.resume = IT7260_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	INIT_DELAYED_WORK(&(ts->work), IT7260_ts_work_func);
	IT7260_wq = create_singlethread_workqueue("IT7260_wq");
	setup_timer(&irq_timer, IT7260_ts_timer_func, 0);

//	queue_delayed_work(IT7260_wq, &ts->work, 0);
	TS_DEBUG("queue applyed, wq=%p, work=%p\n", IT7260_wq, &ts->work);
	if (!IT7260_wq)
	{
		TS_DEBUG("aPply wq failed.\n");
		goto err_check_functionality_failed;
	}

	pr_info("IT7260_ts_probe-client->irq[%d]=\n", client->irq);
	if (client->irq) {
		ret = request_irq(client->irq, IT7260_ts_irq_handler, IRQF_DISABLED,
				client->name, ts);
		pr_info("IT7260_ts_probe-request_irq[%d]=\n", ret);
		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}

	ts->input_dev = input_dev;
	pr_info("=end IT7260_ts_probe=\n");

	return 0;
	err_power_failed: kfree(ts);

	err_check_functionality_failed: return ret;

}

static int IT7260_ts_remove(struct i2c_client *client) {
	return 0;
}

static int IT7260_ts_suspend(struct i2c_client *client, pm_message_t mesg) {
	char ret = 0;
	u8 cmdbuf[] = { 0x04, 0x00, 0x02 };
	uint32_t tmp;

    TS_DEBUG("IT7260_ts_i2c call suspend\n");
	disable_irq_nosync(client->irq);

	imapx_gpio_setcfg(ite7260_int, IG_OUTPUT, IG_NORMAL);
	imapx_gpio_setpin(ite7260_int, 0, IG_NORMAL);

	msleep(100);

#if 0
	if (i2cWriteToIt7260(client, 0x20, cmdbuf, 3) >= 0)
		ret = 0;
	else
		ret = -1;
#endif

	return ret;
}

static int IT7260_ts_resume(struct i2c_client *client) {
	unsigned char ucQuery;
	static int aaa = 0;

#ifdef INT_PIN_OPEN_DRAIN
    //TODO: Here 2 is the pin number of INT pin, so please modify it to the one your system uses.
	gpio_direction_output(2, 0);
	mdelay(10);
#endif //INT_PIN_OPEN_DRAIN
	TS_DEBUG("IT7260_ts_i2c call resume, %d\n", ++aaa);
	it7260_hwinit();
	msleep(100);
	i2cReadFromIt7260(client, 0x80, &ucQuery, 1);
	enable_irq(client->irq);
#ifdef INT_PIN_OPEN_DRAIN
	mdelay(10);
	gpio_direction_output(2, 1);
	mdelay(50);
	gpio_direction_input(2);
#endif //INT_PIN_OPEN_DRAIN
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void IT7260_ts_early_suspend(struct early_suspend *h)
{
	struct IT7260_ts_data *ts;
	ts = container_of(h, struct IT7260_ts_data, early_suspend);
	IT7260_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void IT7260_ts_late_resume(struct early_suspend *h)
{
	struct IT7260_ts_data *ts;
	ts = container_of(h, struct IT7260_ts_data, early_suspend);
	IT7260_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id IT7260_ts_id[] = { { IT7260_I2C_NAME, 0 },
		{ } };

bool IT7260_Init(void) {
	int i;
	int tmp;
	unsigned char ucQuery = 0;
	unsigned char buffer[128];
	struct IT7260_ts_data *ts = gl_ts;

	// Identify Cap Sensor
	do {
		i2cReadFromIt7260(ts->client, 0x80, &ucQuery, 1);
	} while (ucQuery & 0x01);
	buffer[0] = 0x00;
	i2cWriteToIt7260(ts->client, 0x20, buffer, 1);
	do {
		i2cReadFromIt7260(ts->client, 0x80, &ucQuery, 1);
	} while (ucQuery & 0x01);

	memset(&buffer, 0, sizeof(buffer));
	i2cReadFromIt7260(ts->client, 0xA0, buffer, 8);
	pr_info("=IT7260_Init --[%x][%x][%x][%x][%x][%x]=\n", buffer[0], buffer[1],
			buffer[2], buffer[3], buffer[4], buffer[5]);
	if (buffer[1] != 'I' || buffer[2] != 'T' || buffer[3] != 'E') {
		//	return false;
	}

	// Get firmware information
	do {
		i2cReadFromIt7260(ts->client, 0x80, &ucQuery, 1);
	} while (ucQuery & 0x01);
	buffer[0] = 0x01;
	buffer[1] = 0x00;
	i2cWriteToIt7260(ts->client, 0x20, buffer, 2);
	do {
		i2cReadFromIt7260(ts->client, 0x80, &ucQuery, 1);
	} while (ucQuery & 0x01);
	memset(&buffer, 0, sizeof(buffer));
	i2cReadFromIt7260(ts->client, 0xA0, buffer, 8);
	tmp = 0;
	//for (i = 5; i < 9; i++) {
	for (i = 5; i < 8; i++) {
		tmp += buffer[i];
	}
	if (tmp == 0) {
		//	return false;
	}

	//// Reinitialize Firmware
	//set_ite_i2c_nostop(1);
	//do {
	//	ucQuery = i2c_smbus_read_byte_data(ts->client, 0x80);
	//} while (ucQuery & 0x01);
	//buffer[0] = 0x6F;
	//set_ite_i2c_nostop(0);
	//i2c_smbus_write_byte_data(ts->client, 0x20, buffer[0]);

	return true;
}

static struct i2c_driver IT7260_ts_driver = { .probe = IT7260_ts_probe,
		.remove = IT7260_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
		.suspend = IT7260_ts_suspend, .resume = IT7260_ts_resume,
#endif
		.id_table = IT7260_ts_id, .driver = { .name = "IT7260-ts", }, };

struct ite7260_data {
	rwlock_t lock;
	unsigned short bufferIndex;
	unsigned short length;
	unsigned short buffer[MAX_BUFFER_SIZE];
};

int ite7260_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
		unsigned long arg) {
	struct ite7260_data *dev = filp->private_data;
	int retval = 0;
	int i;
	unsigned char ucQuery;
	unsigned char buffer[MAX_BUFFER_SIZE];
	struct ioctl_cmd168 data;
	unsigned char datalen;

	//pr_info("=ite7260_ioctl=\n");
	memset(&data, 0, sizeof(struct ioctl_cmd168));

	switch (cmd) {
	case IOCTL_SET:
		//pr_info("=IOCTL_SET=\n");
		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			retval = -EFAULT;
			goto done;
		}

		if ( copy_from_user(&data, (int __user *)arg, sizeof(struct ioctl_cmd168)) ) {
			retval = -EFAULT;
			goto done;
		}
		buffer[0] = (unsigned char) data.bufferIndex;
		//pr_info("%.2X ", buffer[0]);
		for (i = 1; i < data.length + 1; i++) {
			buffer[i] = (unsigned char) data.buffer[i - 1];
			//pr_info("%.2X ", buffer[i]);
		}

		//pr_info("=================================================\n");
		//pr_info("name[%s]---addr[%x]-flags[%d]=\n",gl_ts->client->name,gl_ts->client->addr,gl_ts->client->flags);
		datalen = (unsigned char) (data.length + 1);
		//pr_info("datalen=%d\n", datalen);
		//write_lock(&dev->lock);
		retval = i2cWriteToIt7260(gl_ts->client,
				(unsigned char) data.bufferIndex, &(buffer[1]), datalen - 1);
		//write_unlock(&dev->lock);
		//pr_info("SET:retval=%x\n", retval);
		retval = 0;
		break;

	case IOCTL_GET:
		//pr_info("=IOCTL_GET=\n");
		if (!access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd))) {
			retval = -EFAULT;
			goto done;
		}

		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			retval = -EFAULT;
			goto done;
		}

		//pr_info("sizeof(struct ioctl_cmd168)=%d\n", sizeof(struct ioctl_cmd168));
		if ( copy_from_user(&data, (int __user *)arg, sizeof(struct ioctl_cmd168)) ) {
			retval = -EFAULT;
			goto done;
		}

		//pr_info("=================================================\n");
		//pr_info("name[%s]---addr[%x]-flags[%d]=\n",gl_ts->client->name,gl_ts->client->addr,gl_ts->client->flags);
		//read_lock(&dev->lock);
		retval = i2cReadFromIt7260(gl_ts->client,
				(unsigned char) data.bufferIndex, (unsigned char*) buffer,
				(unsigned char) data.length);
		//read_unlock(&dev->lock);
		//pr_info("GET:retval=%x\n", retval);
		retval = 0;
		for (i = 0; i < data.length; i++) {
			data.buffer[i] = (unsigned short) buffer[i];
		}
		//pr_info("GET:bufferIndex=%x, dataLength=%d, buffer[0]=%x, buffer[1]=%x, buffer[2]=%x, buffer[3]=%x\n", data.bufferIndex, data.length, buffer[0], buffer[1], buffer[2], buffer[3]);
		//pr_info("GET:bufferIndex=%x, dataLength=%d, buffer[0]=%x, buffer[1]=%x, buffer[2]=%x, buffer[3]=%x\n", data.bufferIndex, data.length, data.buffer[0], data.buffer[1], data.buffer[2], data.buffer[3]);
		//if (data.bufferIndex == 0x80)
		//	data.buffer[0] = 0x00;
		if ( copy_to_user((int __user *)arg, &data, sizeof(struct ioctl_cmd168)) ) {
			retval = -EFAULT;
			goto done;
		}
		break;

	default:
		retval = -ENOTTY;
		break;
	}

	done:
	//pr_info("DONE! retval=%d\n", retval);
	return (retval);
}

int ite7260_open(struct inode *inode, struct file *filp) {
	int i;
	struct ite7260_data *dev;

	pr_info("=ite7260_open=\n");
	dev = kmalloc(sizeof(struct ite7260_data), GFP_KERNEL);
	if (dev == NULL) {
		return -ENOMEM;
	}

	/* initialize members */
	rwlock_init(&dev->lock);
	for (i = 0; i < MAX_BUFFER_SIZE; i++) {
		dev->buffer[i] = 0xFF;
	}

	filp->private_data = dev;

	return 0; /* success */
}

int ite7260_close(struct inode *inode, struct file *filp) {
	struct ite7260_data *dev = filp->private_data;

	if (dev) {
		kfree(dev);
	}

	return 0; /* success */
}

struct file_operations ite7260_fops = { .owner = THIS_MODULE, .open =
		ite7260_open, .release = ite7260_close, .ioctl = ite7260_ioctl, };

int IT7260_dev_create(void)
{
	dev_t dev = MKDEV(ite7260_major, 0);
	int alloc_ret = 0, ret;
	int cdev_err = 0;
	int input_err = 0;
	struct device *class_dev = NULL;
	struct i2c_board_info info;
	struct i2c_client *client;
	struct i2c_adapter *adapter;

	DBG();

	//	if(!IT7260_Init()) {
	//		TS_DEBUG("IT7260 cannot be connected or is in firmware upgrade mode.\n");
	//		goto error;
	//	}

	alloc_ret = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
	if (alloc_ret) {
		TS_DEBUG("IT7260 cdev can't get major number\n");
		goto error;
	}
	ite7260_major = MAJOR(dev);

	// allocate the character device
	cdev_init(&ite7260_cdev, &ite7260_fops);
	ite7260_cdev.owner = THIS_MODULE;
	ite7260_cdev.ops = &ite7260_fops;
	cdev_err = cdev_add(&ite7260_cdev, MKDEV(ite7260_major, ite7260_minor), 1);
	if(cdev_err) {
		goto error;
	}

	// register class
	ite7260_class = class_create(THIS_MODULE, DEVICE_NAME);
	if(IS_ERR(ite7260_class)) {
		TS_DEBUG("Err: failed in creating class.\n");
		goto error;
	}

	ite7260_dev = MKDEV(ite7260_major, ite7260_minor);
	class_dev = device_create(ite7260_class, NULL, ite7260_dev, NULL, DEVICE_NAME);
	if(class_dev == NULL)
	{
		TS_DEBUG("Err: failed in creating device.\n");
		goto error;
	}
	TS_DEBUG("=========================================\n");
	TS_DEBUG("register IT7260 cdev, major: %d, minor: %d \n", ite7260_major, ite7260_minor);
	TS_DEBUG("=========================================\n");

	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		input_err = -ENOMEM;
		printk(KERN_ERR "IT7260_ts_probe: Failed to allocate input device\n");
		goto error;
	}
	input_dev->name = "IT7260";
	input_dev->phys = "I2C";
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x7260;

	//input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	//set_bit(EV_SYN, input_dev->evbit);
	//set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(KEY_HOME, input_dev->keybit);
	set_bit(KEY_SEARCH, input_dev->keybit);
	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_MENU, input_dev->keybit);
	//set_bit(BTN_TOUCH, input_dev->keybit);
	//set_bit(BTN_2, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, 1024, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, 1024, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 2, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 2, 0, 0);
	input_set_abs_params(input_dev, ABS_X, 0, 1024, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, 1024, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 1, 0, 0);
	input_err = input_register_device(input_dev);
	if (input_err) goto error;
	pr_info("it7260 driver is on###############################################################\n");

	return 0;

	error:
	if(cdev_err == 0) {
		cdev_del(&ite7260_cdev);
	}
	if(alloc_ret == 0) {
		unregister_chrdev_region(dev, 1);
	}
	if(input_dev) {
		input_free_device(input_dev);
	}
	if (IT7260_wq)
	destroy_workqueue(IT7260_wq);

	return -1;
}

static int __devinit IT7260_ts_init(void) {
	int ret;
	struct i2c_board_info info;
	struct i2c_client *client;
	struct i2c_adapter *adapter;

	ret = i2c_add_driver(&IT7260_ts_driver);
	if(ret)
	  printk(KERN_ERR "Add driver IT7260 failed.\n");

	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = 0x46;
	info.irq = IRQ_GPIO;
	strlcpy(info.type, IT7260_I2C_NAME, strlen(IT7260_I2C_NAME) + 1);
	adapter = i2c_get_adapter(CONFIG_TP_ITE7260_I2C + 1);

	if(!adapter)
	  printk(KERN_ERR "ITE7260 get iic bus 0 failed.\n");
	client = i2c_new_device(adapter, &info);
	if(!client)
	  printk(KERN_ERR "Can't add i2c device at 0x%x\n", (uint32_t)info.addr);

//	IT7260_sx8651_lite_init();
//	IT7260_dev_create();
	return 0;
}

static void __exit IT7260_ts_exit(void) {
	dev_t dev = MKDEV(ite7260_major, ite7260_minor);

	// unregister class
	device_destroy(ite7260_class, ite7260_dev);
	class_destroy(ite7260_class);

	// unregister driver handle
	cdev_del(&ite7260_cdev);
	unregister_chrdev_region(dev, 1);

	i2c_del_driver(&IT7260_ts_driver);
	if (IT7260_wq)
	destroy_workqueue(IT7260_wq);
}

module_init( IT7260_ts_init);
module_exit( IT7260_ts_exit);

MODULE_DESCRIPTION("IT7260 Touchscreen Driver");
MODULE_LICENSE("GPL");
