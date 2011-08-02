/*
 *
 * Touch Screen I2C Driver for EETI Controller
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

// Release Date: 2010/12/27
// Based on 2010/11/08 and add loopback function 

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/freezer.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <linux/poll.h>
#include <linux/kfifo.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/irq.h>

#define DEBUG
#ifdef DEBUG
	#define TS_DEBUG(fmt,args...)  printk( KERN_ERR "[egalax_i2c]: " fmt, ## args)
	#define DBG() printk("[%s]:%d => \n",__FUNCTION__,__LINE__)
#else
	#define TS_DEBUG(fmt,args...)
	#define DBG()
#endif

//#define _NON_INPUT_DEV // define this to disable register input device	

static int global_major = 0; // dynamic major by default 
static int global_minor = 0;

#define MAX_I2C_LEN		10
#define FIFO_SIZE		PAGE_SIZE
#define MAX_SUPPORT_POINT	5
#define REPORTID_MOUSE		0x01
#define REPORTID_VENDOR		0x03
#define REPORTID_MTOUCH		0x04

/// ioctl command ///
#define EGALAX_IOC_MAGIC	0x72
#define	EGALAX_IOCWAKEUP	_IO(EGALAX_IOC_MAGIC, 1)
#define EGALAX_IOC_MAXNR	1

/// delays ///
#define EGALAX_JUNK_DELAY	22222222
#define EGALAX_POINT_DELAY	0

struct point_data {
	short Status;
	short X;
	short Y;
};

struct _egalax_i2c {
	struct workqueue_struct *ktouch_wq;
	struct delayed_work work;
	struct mutex mutex_wq;
	struct i2c_client *client;
	struct hrtimer timer;
	char work_state;
	char skip_packet;
};

struct egalax_char_dev
{
	int OpenCnts;
	struct cdev cdev;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
	struct kfifo* pDataKFiFo;
#else
	struct kfifo DataKFiFo;
#endif
	unsigned char *pFiFoBuf;
	spinlock_t FiFoLock;
	struct semaphore sem;
	wait_queue_head_t fifo_inq;
};

unsigned int egl7200_int;
static long unsigned int g_pcnt = 0;
static uint32_t g_pjmp = 0;
static struct _egalax_i2c *p_egalax_i2c_dev = NULL;	// allocated in egalax_i2c_probe
static struct egalax_char_dev *p_char_dev = NULL;	// allocated in init_module
static atomic_t egalax_char_available = ATOMIC_INIT(1);
static struct class *egalax_class;
#ifndef _NON_INPUT_DEV
static struct input_dev *input_dev = NULL;
static struct point_data PointBuf[MAX_SUPPORT_POINT];
#endif //#ifndef _NON_INPUT_DEV

/* code for imapx220 */
static int egalax_int_out(void) {
	/* not supported for EINT1 */
	return 0;
}

static int egalax_int_in(void) {
	/* not supported for EINT1 */
	return 0;
}

static int egalax_int_val(void) {
	return !!(imapx_gpio_getpin(egl7200_int, IG_NORMAL));
}

static void egalax_int_init(void) {
	imapx_gpio_setirq(egl7200_int, FILTER_MAX, IG_LOW, 1);
}

static int egalax_new_imapdev(void)
{
	struct i2c_client *client;
	struct i2c_adapter *adapter;
	struct i2c_board_info info = {
		.addr	= 0x04,
		//.irq	= IRQ_EINT1,
		.type	= "egalax_i2c",
	};

	egl7200_int = __imapx_name_to_gpio(CONFIG_TP_EGL7200_INT);
	if(egl7200_int == IMAPX_GPIO_ERROR) {
		printk(KERN_ERR "failed to get egl7200_int pin.\n");
		return -1;
	}
	info.irq = imapx_gpio_to_irq(egl7200_int);

	adapter = i2c_get_adapter(CONFIG_TP_EGL7200_I2C + 1);

	if(!adapter)
	{
		printk(KERN_ERR "egalax get i2c bus 0 failed.\n");
		return -1;
	}

	/* init interrupt */
	egalax_int_init();

	client = i2c_new_device(adapter, &info);
	if(!client)
	{
		printk(KERN_ERR "Can't add i2c device at 0x%x\n", (uint32_t)info.addr);
		return -1;
	}

	return 0;
}
/* END: code for imapx220 */

static int egalax_cdev_open(struct inode *inode, struct file *filp)
{
	struct egalax_char_dev *cdev;

	DBG();

	cdev = container_of(inode->i_cdev, struct egalax_char_dev, cdev);
	if( cdev == NULL )
	{
        	TS_DEBUG(" No such char device node \n");
		return -ENODEV;
	}
	
	if( !atomic_dec_and_test(&egalax_char_available) )
	{
		atomic_inc(&egalax_char_available);
		return -EBUSY; /* already open */
	}

	cdev->OpenCnts++;
	filp->private_data = cdev;// Used by the read and write metheds

	TS_DEBUG(" egalax_cdev_open done \n");
	try_module_get(THIS_MODULE);
	return 0;
}

static int egalax_cdev_release(struct inode *inode, struct file *filp)
{
	struct egalax_char_dev *cdev; // device information

	DBG();

	cdev = container_of(inode->i_cdev, struct egalax_char_dev, cdev);
        if( cdev == NULL )
        {
                TS_DEBUG(" No such char device node \n");
                return -ENODEV;
        }

	atomic_inc(&egalax_char_available); /* release the device */

	filp->private_data = NULL;
	cdev->OpenCnts--;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
	kfifo_reset( cdev->pDataKFiFo );
#else
	kfifo_reset( &cdev->DataKFiFo );
#endif

	TS_DEBUG(" egalax_cdev_release done \n");
	module_put(THIS_MODULE);
	return 0;
}

#define MAX_READ_BUF_LEN	50
static char fifo_read_buf[MAX_READ_BUF_LEN];
static ssize_t egalax_cdev_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	int read_cnt, ret, fifoLen;
	struct egalax_char_dev *cdev = file->private_data;

	DBG();
	
	if( down_interruptible(&cdev->sem) )
		return -ERESTARTSYS;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
	fifoLen = kfifo_len(cdev->pDataKFiFo);
#else
	fifoLen = kfifo_len(&cdev->DataKFiFo);
#endif

	while( fifoLen<1 ) /* nothing to read */
	{
		up(&cdev->sem); /* release the lock */
		if( file->f_flags & O_NONBLOCK )
			return -EAGAIN;

	#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
		if( wait_event_interruptible(cdev->fifo_inq, kfifo_len( cdev->pDataKFiFo )>0) )
	#else
		if( wait_event_interruptible(cdev->fifo_inq, kfifo_len( &cdev->DataKFiFo )>0) )
	#endif
		{
			return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
		}

		if( down_interruptible(&cdev->sem) )
			return -ERESTARTSYS;
	}

	if(count > MAX_READ_BUF_LEN)
		count = MAX_READ_BUF_LEN;

	TS_DEBUG("\"%s\" reading: real fifo data\n", current->comm);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
	read_cnt = kfifo_get(cdev->pDataKFiFo, fifo_read_buf, count);
#else
	read_cnt = kfifo_out_locked(&cdev->DataKFiFo, fifo_read_buf, count, &cdev->FiFoLock);
#endif

	ret = copy_to_user(buf, fifo_read_buf, read_cnt)?-EFAULT:read_cnt;

	up(&cdev->sem);
	
	return ret;
}

static ssize_t egalax_cdev_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	char *tmp;
	struct egalax_char_dev *cdev = file->private_data;
	int ret=0;

	DBG();

	if( down_interruptible(&cdev->sem) )
		return -ERESTARTSYS;

	if (count > MAX_I2C_LEN)
		count = MAX_I2C_LEN;

	tmp = kmalloc(count,GFP_KERNEL);
	if(tmp==NULL)
	{
		up(&cdev->sem);
		return -ENOMEM;
	}

	if(copy_from_user(tmp, buf, count))
	{
		up(&cdev->sem);
		kfree(tmp);
		return -EFAULT;
	}
	
	ret = i2c_master_send(p_egalax_i2c_dev->client, tmp, count);
	TS_DEBUG("I2C writing %zu bytes.\n", count);

	kfree(tmp);

	up(&cdev->sem);

	return ret;
}

static int wakeup_controller(void)
{
	int ret=0;

	egalax_int_out();
	udelay(10);
	egalax_int_in();
	printk(KERN_ERR "[egalax_i2c]: INT wakeup touch controller done\n");
	
	return ret;
}

static int egalax_cdev_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long args)
{	
	//struct egalax_char_dev *cdev = file->private_data;
	int ret=0;

	if(_IOC_TYPE(cmd) != EGALAX_IOC_MAGIC)
		return -ENOTTY;
	if(_IOC_NR(cmd) > EGALAX_IOC_MAXNR)
		return -ENOTTY;

	if(_IOC_DIR(cmd) & _IOC_READ)
		ret = !access_ok(VERIFY_WRITE, (void __user*)args, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		ret = !access_ok(VERIFY_READ, (void __user*)args, _IOC_SIZE(cmd));

	if(ret)
		return -EFAULT;

	//printk(KERN_ERR "Handle device ioctl command\n");
	switch (cmd)
	{
		case EGALAX_IOCWAKEUP:
			ret = wakeup_controller();
			break;
		default:
			ret = -ENOTTY;
			break;
	}

	return ret;
}

static unsigned int egalax_cdev_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct egalax_char_dev *cdev = filp->private_data;
	unsigned int mask = 0;
	int fifoLen;
	
	down(&cdev->sem);
	poll_wait(filp, &cdev->fifo_inq,  wait);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
	fifoLen = kfifo_len(cdev->pDataKFiFo);
#else
	fifoLen = kfifo_len(&cdev->DataKFiFo);
#endif

	if( fifoLen > 0 )
		mask |= POLLIN | POLLRDNORM;    /* readable */
	if( (FIFO_SIZE - fifoLen) > MAX_I2C_LEN )
		mask |= POLLOUT | POLLWRNORM;   /* writable */

	up(&cdev->sem);
	return mask;
}

#ifndef _NON_INPUT_DEV
static int LastUpdateID = 0;
static void ProcessReport(unsigned char *buf, int buflen)
{
	int i;
	short X=0, Y=0, ContactID=0, Status=0;

	if(buflen!=MAX_I2C_LEN || buf[0]!=0x04) // check buffer len & header
	{
		TS_DEBUG("err pkt: Len=%d header=%x\n", buflen, buf[0]);
		return;
	}

	Status = buf[1]&0x01;
	ContactID = (buf[1]&0x7C)>>2;
	X = ((buf[3]<<8) + buf[2])>>4;
	Y = ((buf[5]<<8) + buf[4])>>4;
	
	PointBuf[ContactID].Status = Status;
	PointBuf[ContactID].X = X;
	PointBuf[ContactID].Y = Y;
	LastUpdateID = ContactID;
//	TS_DEBUG("[%d]: (%d, %d)%d\n", ContactID, X, Y, Status);

	/* Only report 1/4 points or finger ups*/
	g_pjmp = (g_pjmp + 1) & 0x3;
	if((g_pjmp != 1) && Status)
	{
//		TS_DEBUG("point jumped\n");
		return ;
	}

	g_pcnt++;
	if(!(g_pcnt & 0xff))
	  TS_DEBUG("p cnt: %lu\n", g_pcnt);

	// Send point report
	if( !Status || (ContactID <= LastUpdateID) )
	{
		for(i=0; i<MAX_SUPPORT_POINT;i++)
		{
			if(PointBuf[i].Status >= 0)
			{
				input_report_abs(input_dev, ABS_MT_TRACKING_ID, i);			
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, PointBuf[i].Status);
				input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 0);
				input_report_abs(input_dev, ABS_MT_POSITION_X, PointBuf[i].X);
				input_report_abs(input_dev, ABS_MT_POSITION_Y, PointBuf[i].Y);

				input_mt_sync(input_dev);

				if(PointBuf[i].Status == 0)
					PointBuf[i].Status--;
			}
		}
		input_sync(input_dev);
//		TS_DEBUG("Input sync point data done!\n");
	}
}

static struct input_dev * allocate_Input_Dev(void)
{
	int ret;
	struct input_dev *pInputDev=NULL;

	pInputDev = input_allocate_device();
	if(pInputDev == NULL)
	{
		TS_DEBUG("Failed to allocate input device\n");
		return NULL;//-ENOMEM;
	}

	pInputDev->name = "eGalax Touch Screen";
	pInputDev->phys = "I2C";
	pInputDev->id.bustype = BUS_I2C;
	pInputDev->id.vendor = 0x0EEF;
	pInputDev->id.product = 0x0020;
	
	set_bit(EV_ABS, pInputDev->evbit);

	input_set_abs_params(pInputDev, ABS_MT_POSITION_X, 0, 2047, 0, 0);
	input_set_abs_params(pInputDev, ABS_MT_POSITION_Y, 0, 2047, 0, 0);
	input_set_abs_params(pInputDev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(pInputDev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(pInputDev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);

	ret = input_register_device(pInputDev);
	if(ret) 
	{
		TS_DEBUG("Unable to register input device.\n");
		input_free_device(pInputDev);
		return NULL;
	}
	
	return pInputDev;
}
#endif //#ifndef _NON_INPUT_DEV

static int egalax_i2c_measure(struct i2c_client *client, char skip_packet)
{
	u8 x_buf[MAX_I2C_LEN];
	int count, loop=3;
	
//	DBG();

	do{
		count = i2c_master_recv(client, x_buf, MAX_I2C_LEN);
	}while(count == -EAGAIN && --loop);

	if( count<0 || (x_buf[0]!=REPORTID_VENDOR && x_buf[0]!=REPORTID_MTOUCH) )
	{
//		TS_DEBUG("I2C read error data with Len=%d hedaer=%d\n", count, x_buf[0]);
		return -1;
	}

//	TS_DEBUG("egalax_i2c read data with Len=%d header=%x\n", count, x_buf[0]);
	if(x_buf[0]==REPORTID_VENDOR)
		TS_DEBUG("egalax_i2c get command packet, packet=> "
		   "0x%x|0x%x|0x%x|0x%x|0x%x|0x%x|0x%x|0x%x|0x%x|0x%x\n", x_buf[0],
		   x_buf[1], x_buf[2], x_buf[3], x_buf[4], x_buf[5],
		   x_buf[6], x_buf[7], x_buf[8], x_buf[9]);

	if( skip_packet > 0 )
	{
		TS_DEBUG("skiping package: %d\n", skip_packet);
		return count;
	}

#ifndef _NON_INPUT_DEV
	if( count>0 && x_buf[0]==REPORTID_MTOUCH )
	{
		ProcessReport(x_buf, count);

		return count;
	}
#endif //#ifndef _NON_INPUT_DEV

	if( count>0 && p_char_dev->OpenCnts>0 ) // If someone reading now! put the data into the buffer!
	{
	#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
		kfifo_put(p_char_dev->pDataKFiFo, x_buf, count);
	#else
		kfifo_in_locked(&p_char_dev->DataKFiFo, x_buf, count, &p_char_dev->FiFoLock);
	#endif
	 	wake_up_interruptible( &p_char_dev->fifo_inq );
	}

	return count;
}

static enum hrtimer_restart
egalax_i2c_timer(struct hrtimer *handle)
{
	struct i2c_client *client = p_egalax_i2c_dev->client;
	if( p_egalax_i2c_dev->work_state > 0 )
		enable_irq(client->irq);

	return HRTIMER_NORESTART;
}

static void egalax_i2c_wq(struct work_struct *work)
{
	struct _egalax_i2c *egalax_i2c = container_of(work, struct _egalax_i2c, work);
	struct i2c_client *client = egalax_i2c->client;
	int ret = 0;

//	TS_DEBUG("egalax_i2c_wq run\n");

	mutex_lock(&egalax_i2c->mutex_wq);

	/*continue recv data*/
	if(!egalax_int_val() && egalax_i2c->work_state > 0)
	{
		ret = egalax_i2c_measure(client, egalax_i2c->skip_packet);

		/* junk data, delay 20ms */
		hrtimer_start(&egalax_i2c->timer, ktime_set(0, EGALAX_POINT_DELAY),
		   HRTIMER_MODE_REL);
	} else
		hrtimer_start(&egalax_i2c->timer,
		   ktime_set(0, 20), HRTIMER_MODE_REL);
	
	if( egalax_i2c->skip_packet > 0 )
		egalax_i2c->skip_packet = 0;

	mutex_unlock(&egalax_i2c->mutex_wq);

#if 0
	if(!egalax_int_val() && egalax_i2c->work_state > 0)
	{
		queue_delayed_work(egalax_i2c->ktouch_wq, &egalax_i2c->work, 25);
		return ;
	}
#endif
//	TS_DEBUG("egalax_i2c_wq leave\n");
}

static irqreturn_t egalax_i2c_interrupt(int irq, void *dev_id)
{
	struct _egalax_i2c *egalax_i2c = (struct _egalax_i2c *)dev_id;
	uint32_t tmp;

//	TS_DEBUG("egalax_i2c_interrupt with irq:%d\n", irq);

//	TS_DEBUG("irq: %d\n", egalax_int_val());

	tmp = readl(rSRCPND);
	tmp |= (0x1 << 1);   
	writel(tmp,rSRCPND); 
	
	disable_irq_nosync(irq);
	queue_delayed_work(egalax_i2c->ktouch_wq, &egalax_i2c->work, 0);

	return IRQ_HANDLED;
}

#ifdef CONFIG_PM
static int egalax_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct _egalax_i2c *egalax_i2c = i2c_get_clientdata(client);
	u8 cmdbuf[MAX_I2C_LEN]={0x03, 0x05, 0x0A, 0x03, 0x36, 0x3F, 0x02, 0, 0, 0};
	
	i2c_master_send(client, cmdbuf, MAX_I2C_LEN);

	egalax_i2c->work_state = 0;
	disable_irq(client->irq);
	cancel_work_sync(&egalax_i2c->work);

	printk(KERN_DEBUG "[egalax_i2c]: device suspend done\n");	

	if(device_may_wakeup(&client->dev)) 
	{
		enable_irq_wake(client->irq);
	}
	else 
	{
		printk(KERN_DEBUG "[egalax_i2c]: device_may_wakeup false\n");
	}

	return 0;
}

static int egalax_i2c_resume(struct i2c_client *client)
{
	struct _egalax_i2c *egalax_i2c = i2c_get_clientdata(client);
	
	if(device_may_wakeup(&client->dev)) 
	{
		disable_irq_wake(client->irq);
	}
	else 
	{
		printk(KERN_DEBUG "[egalax_i2c]: device_may_wakeup false\n");
	}

	wakeup_controller();
	egalax_i2c->work_state = 1;
	enable_irq(client->irq);

	printk(KERN_DEBUG "[egalax_i2c]: device wakeup done\n");

	return 0;
}
#else
#define egalax_i2c_suspend       NULL
#define egalax_i2c_resume        NULL
#endif

static void sendLoopback(struct i2c_client *client)
{
	u8 cmdbuf[MAX_I2C_LEN]={0x03, 0x03, 0x0A, 0x01, 0x44, 0, 0, 0, 0, 0};
	i2c_master_send(client, cmdbuf, MAX_I2C_LEN);
}

static int __devinit egalax_i2c_probe(struct i2c_client *client,
   const struct i2c_device_id *id)
{
	int ret;

	DBG();
	printk(KERN_DEBUG "[egalax_i2c]: start probe\n");

	p_egalax_i2c_dev = (struct _egalax_i2c *)kzalloc(sizeof(struct _egalax_i2c), GFP_KERNEL);
	if (!p_egalax_i2c_dev) 
	{
		printk(KERN_ERR "[egalax_i2c]: request memory failed\n");
		ret = -ENOMEM;
		goto fail1;
	}

#ifndef _NON_INPUT_DEV
	input_dev = allocate_Input_Dev();
	if(input_dev==NULL)
	{
		printk(KERN_ERR "[egalax_i2c]: allocate_Input_Dev failed\n");
		ret = -EINVAL; 
		goto fail2;
	}
	TS_DEBUG("egalax_i2c register input device done\n");
	memset(PointBuf, 0, sizeof(struct point_data)*MAX_SUPPORT_POINT);
#endif //#ifndef _NON_INPUT_DEV

	p_egalax_i2c_dev->client = client;
	mutex_init(&p_egalax_i2c_dev->mutex_wq);

	p_egalax_i2c_dev->ktouch_wq = create_workqueue("egalax_touch_wq"); 
	INIT_DELAYED_WORK(&p_egalax_i2c_dev->work, egalax_i2c_wq);
	
	hrtimer_init(&p_egalax_i2c_dev->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	p_egalax_i2c_dev->timer.function = egalax_i2c_timer;

	i2c_set_clientdata(client, p_egalax_i2c_dev);

	sendLoopback(client);

	if( egalax_int_val() )
		p_egalax_i2c_dev->skip_packet = 0;
	else
		p_egalax_i2c_dev->skip_packet = 0;

	p_egalax_i2c_dev->work_state = 1;
	ret = request_irq(client->irq, egalax_i2c_interrupt, IRQF_DISABLED,
		 client->name, p_egalax_i2c_dev);
	if( ret ) 
	{
		printk(KERN_ERR "[egalax_i2c]: request irq(%d) failed\n", client->irq);
		goto fail3;
	}
	TS_DEBUG("egalax_i2c request irq(%d) with result:%d\n", client->irq, ret);

#ifdef CONFIG_PM
	device_init_wakeup(&client->dev, 1);
#endif

	printk(KERN_DEBUG "[egalax_i2c]: probe done\n");
	return 0;

fail3:
	i2c_set_clientdata(client, NULL);
	destroy_workqueue(p_egalax_i2c_dev->ktouch_wq); 
	free_irq(client->irq, p_egalax_i2c_dev);
#ifndef _NON_INPUT_DEV
	input_unregister_device(input_dev);
	input_free_device(input_dev);
	input_dev = NULL;
#endif //#ifndef _NON_INPUT_DEV
fail2:
fail1:
	kfree(p_egalax_i2c_dev);
	p_egalax_i2c_dev = NULL;

	printk(KERN_DEBUG "[egalax_i2c]: probe failed\n");
	return ret;
}

static int __devexit egalax_i2c_remove(struct i2c_client *client)
{
	struct _egalax_i2c *egalax_i2c = i2c_get_clientdata(client);

	DBG();

#ifndef _NON_INPUT_DEV
	if(input_dev)
	{
		TS_DEBUG("unregister input device\n");
		input_unregister_device(input_dev);
		input_free_device(input_dev);
		input_dev = NULL;
	}
#endif //#ifndef _NON_INPUT_DEV

	if(p_egalax_i2c_dev->ktouch_wq) 
	{
		destroy_workqueue(p_egalax_i2c_dev->ktouch_wq); 
	}

	if(client->irq)
	{
		free_irq(client->irq, egalax_i2c);
	}

	i2c_set_clientdata(client, NULL);
	kfree(egalax_i2c);
	p_egalax_i2c_dev = NULL;

	return 0;
}

static struct i2c_device_id egalax_i2c_idtable[] = { 
	{ "egalax_i2c", 0 }, 
	{ } 
}; 

MODULE_DEVICE_TABLE(i2c, egalax_i2c_idtable);

static struct i2c_driver egalax_i2c_driver = {
	.driver = {
		.name 	= "egalax_i2c",
	},
	.id_table	= egalax_i2c_idtable,
	.probe		= egalax_i2c_probe,
	.remove		= __devexit_p(egalax_i2c_remove),
	.suspend	= egalax_i2c_suspend,
	.resume		= egalax_i2c_resume,
};

static const struct file_operations egalax_cdev_fops = {
	.owner	= THIS_MODULE,
	.read	= egalax_cdev_read,
	.write	= egalax_cdev_write,
	.ioctl	= egalax_cdev_ioctl,
	.poll	= egalax_cdev_poll,
	.open	= egalax_cdev_open,
	.release= egalax_cdev_release,
};

static void egalax_i2c_ts_exit(void)
{
	dev_t devno = MKDEV(global_major, global_minor);
	DBG();

	if(p_char_dev)
	{
		TS_DEBUG("unregister character device\n");
		if( p_char_dev->pFiFoBuf )
			kfree(p_char_dev->pFiFoBuf);
	
		cdev_del(&p_char_dev->cdev);
		kfree(p_char_dev);
		p_char_dev = NULL;
	}

	unregister_chrdev_region( devno, 1);

	if(!IS_ERR(egalax_class))
	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
		class_device_destroy(egalax_class, devno);
#else
		device_destroy(egalax_class, devno);
#endif 
		class_destroy(egalax_class);
	}
	
	i2c_del_driver(&egalax_i2c_driver);

	printk(KERN_DEBUG "[egalax_i2c]: driver exit\n");
}

static struct egalax_char_dev* setup_chardev(dev_t dev)
{
	struct egalax_char_dev *pCharDev;
	int result;

	pCharDev = kmalloc(1*sizeof(struct egalax_char_dev), GFP_KERNEL);
	if(!pCharDev) 
		goto fail_cdev;
	memset(pCharDev, 0, sizeof(struct egalax_char_dev));

	spin_lock_init( &pCharDev->FiFoLock );
	pCharDev->pFiFoBuf = kmalloc(sizeof(unsigned char)*FIFO_SIZE, GFP_KERNEL);
	if(!pCharDev->pFiFoBuf)
		goto fail_fifobuf;
	memset(pCharDev->pFiFoBuf, 0, sizeof(unsigned char)*FIFO_SIZE);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
	pCharDev->pDataKFiFo = kfifo_init(pCharDev->pFiFoBuf, FIFO_SIZE, GFP_KERNEL, &pCharDev->FiFoLock);
	if( pCharDev->pDataKFiFo==NULL )
		goto fail_kfifo;
#else
	kfifo_init(&pCharDev->DataKFiFo, pCharDev->pFiFoBuf, FIFO_SIZE);
	if( !kfifo_initialized(&pCharDev->DataKFiFo) )
		goto fail_kfifo;
#endif
	
	pCharDev->OpenCnts = 0;
	cdev_init(&pCharDev->cdev, &egalax_cdev_fops);
	pCharDev->cdev.owner = THIS_MODULE;
	sema_init(&pCharDev->sem, 1);
	init_waitqueue_head(&pCharDev->fifo_inq);

	result = cdev_add(&pCharDev->cdev, dev, 1);
	if(result)
	{
		TS_DEBUG(KERN_ERR "Error cdev ioctldev added\n");
		goto fail_kfifo;
	}

	return pCharDev; 

fail_kfifo:
	kfree(pCharDev->pFiFoBuf);
fail_fifobuf:
	kfree(pCharDev);
fail_cdev:
	return NULL;
}

static int egalax_i2c_ts_init(void)
{
	int result;
	dev_t devno = 0;

	DBG();

	// Asking for a dynamic major unless directed otherwise at load time.
	if(global_major) 
	{
		devno = MKDEV(global_major, global_minor);
		result = register_chrdev_region(devno, 1, "egalax_i2c");
	} 
	else 
	{
		result = alloc_chrdev_region(&devno, global_minor, 1, "egalax_i2c");
		global_major = MAJOR(devno);
	}

	if (result < 0)
	{
		TS_DEBUG(" egalax_i2c cdev can't get major number\n");
		return 0;
	}

	// allocate the character device
	p_char_dev = setup_chardev(devno);
	if(!p_char_dev) 
	{
		result = -ENOMEM;
		goto fail;
	}

	egalax_class = class_create(THIS_MODULE, "egalax_i2c");
	if(IS_ERR(egalax_class))
	{
		TS_DEBUG("Err: failed in creating class.\n");
		result = -EFAULT;
		goto fail;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
	class_device_create(egalax_class, NULL, devno, NULL, "egalax_i2c");
#else
	device_create(egalax_class, NULL, devno, NULL, "egalax_i2c");
#endif
	TS_DEBUG("register egalax_i2c cdev, major: %d \n",global_major);

	printk(KERN_DEBUG "[egalax_i2c]: init done\n");

	/* Here add the device and driver */
	egalax_new_imapdev();
	return i2c_add_driver(&egalax_i2c_driver);

fail:	
	egalax_i2c_ts_exit();
	return result;
}

module_init(egalax_i2c_ts_init);
module_exit(egalax_i2c_ts_exit);

MODULE_DESCRIPTION("egalax touch screen i2c driver");
MODULE_LICENSE("GPL");

