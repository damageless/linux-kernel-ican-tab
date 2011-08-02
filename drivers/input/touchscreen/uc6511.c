#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <mach/imapx_gpio.h>
#include <asm/io.h>
#include <linux/gpio.h>


struct timer_list irq_timer;
struct hrtimer ts_timer;

uint32_t g_coutn = 0;
uint32_t g_dvd = 0;


#define PRINT_XY		0
#define MAX_TRACK_POINT		5

#define	MAX_12BIT			((1<<12)-1)

#define UC6511_DEVID		0x0A

/* for 7901ui */
//#define UC6511_IRQN			IRQ_EINT5

/* for 0702ab */
unsigned int uc6511_int;
unsigned int uc6511_irq_no;
//#define UC6511_IRQN			IRQ_EINT1

typedef struct i2c_client	bus_device;
static spinlock_t ts_spin = SPIN_LOCK_UNLOCKED;


static u16 fingers_x[MAX_TRACK_POINT];
static u16 fingers_y[MAX_TRACK_POINT];

struct uc6511_point {
	char			count;
	char			number;
	u16			x;
	u16			y;
};


struct uc6511 {
	bus_device		*bus;
	struct input_dev	*input;
	struct work_struct	work;
	struct delayed_work dwork;
	struct timer_list	timer;

	struct mutex		mutex;
	unsigned		disabled:1;	/* P: mutex */

	char			phys[32];
};
struct uc6511 *uc_ts;

static enum hrtimer_restart uc6511_timer_func(struct hrtimer * handle){ 
	struct uc6511 *ts = uc_ts;              
	uint32_t tmp;
//	printk(KERN_INFO "enable irq1 \n");
/*
	tmp = readl(rSRCPND);
	tmp |= (0x1 << 1);   
	writel(tmp,rSRCPND); 
	tmp = readl(rINTPND);
	tmp |= (0x1 << 1);   
	writel(tmp,rINTPND); 
*/	
	enable_irq(uc6511_irq_no);            
	return HRTIMER_NORESTART;

}

static void uc6511_work(struct work_struct *work)
{
	struct uc6511_point pt;
	char point_count;
	char point_no;
	int x, y, tx = 0, ty = 0, tn = 0;

	struct uc6511 *ts = container_of(work, struct uc6511, work);
	struct input_dev *input_dev = ts->input;

//	printk(KERN_ERR "_%s_\n", __func__);
//	disable_irq_nosync(UC6511_IRQN);
	i2c_master_recv(ts->bus, &pt, 6);
	point_count = pt.count;
	point_no    = pt.number;

//	**** calculate x/y sampling values
	x = MAX_12BIT - swab16(pt.y);
	y = swab16(pt.x);

	fingers_x[point_no - 1] = x;
	fingers_y[point_no - 1] = y;

#if PRINT_XY
	printk(KERN_INFO "uc6511_work %d / %d +, %x\n", point_count, point_no, readl(rEINTCON));
#endif

	if (point_count == point_no) {
#if PRINT_XY
		printk(KERN_INFO "%d / %d", point_count, point_no);
#endif

#if 1
		g_dvd = (g_dvd + 1) % 3;
		if(g_dvd != 1)
		  /* jump the odd */
		  return;
#endif

		for (point_no = 0; point_no < point_count; point_no ++) {

#if 0
			if((tn == point_no) && (tx == fingers_x[point_no]) &&
			   (ty == fingers_y[point_no]))
			  continue;
#endif
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 128);
			input_report_abs(input_dev, ABS_MT_POSITION_X, fingers_x[point_no]);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, fingers_y[point_no]);

			input_mt_sync(input_dev);
//			printk(KERN_ERR "%d: (%d, %d)",
//			   point_no, fingers_x[point_no], fingers_y[point_no]);
#if PRINT_XY
			printk(KERN_INFO "(%d, %d)", fingers_x[point_no], fingers_y[point_no]);
#endif
		}
		input_sync(input_dev);
	}
	else if (point_count == 0x80) {
		printk(KERN_INFO "up\n");
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_mt_sync(input_dev);
		input_sync(input_dev);
	}

//	msleep(10);
//	enable_irq(UC6511_IRQN);
//	mod_timer(&irq_timer, jiffies + msecs_to_jiffies(10));
//	hrtimer_start(&ts_timer, ktime_set(0, 10000000), HRTIMER_MODE_REL);

}

static irqreturn_t uc6511_irq(int irq, void *handle)
{
	struct uc6511 *ts = handle;
	uint32_t      tmp;
	unsigned long flags;

	g_coutn++;

#if 0
	printk(KERN_ERR "ii\n");
	if((g_coutn & 0x1))
	{
		printk(KERN_ERR "oi\n");
		return IRQ_HANDLED;
	}
#endif

//	printk(KERN_ERR "hi\n");
	if(!(g_coutn & 0xff))
	  printk(KERN_ERR "irq += %lu\n", g_coutn);
//	spin_lock_irqsave(&ts_spin, flags);
	/* The repeated conversion sequencer controlled by TMR kicked off too fast.
	 * We ignore the last and process the sample sequence currently in the queue.
	 * It can't be older than 9.4ms
	 */
//	printk(KERN_INFO "uc6511-get-irq\n");

//**************************
	//really need here????
	tmp = readl(rSRCPND);
	tmp |= (0x1 << 1);   
	writel(tmp,rSRCPND); 
//*************************

//	disable_irq_nosync(UC6511_IRQN);

//	if (!work_pending(&ts->work))
		schedule_work(&ts->work);
//		schedule_delayed_work(&ts->dwork, 3);

//	spin_unlock_irqrestore(&ts_spin, flags);

	return IRQ_HANDLED;
}

static void uc6511_disable(struct uc6511 *ts)
{
	mutex_lock(&ts->mutex);

	printk(KERN_ERR "DISABLE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!########\n");
	if (!ts->disabled) {

		ts->disabled = 1;
		disable_irq(ts->bus->irq);

		cancel_work_sync(&ts->work);
	}

	mutex_unlock(&ts->mutex);
}

static void uc6511_enable(struct uc6511 *ts)
{
	mutex_lock(&ts->mutex);

	if (ts->disabled) {
		ts->disabled = 0;
//		enable_irq(ts->bus->irq);
	}

	mutex_unlock(&ts->mutex);
}

static ssize_t uc6511_pen_down_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return 0;
}

static DEVICE_ATTR(pen_down, 0644, uc6511_pen_down_show, NULL);

static ssize_t uc6511_disable_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct uc6511 *ts = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ts->disabled);
}

static ssize_t uc6511_disable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct uc6511 *ts = dev_get_drvdata(dev);
	unsigned long val;
	int error;

	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	if (val)
		uc6511_disable(ts);
	else
		uc6511_enable(ts);

	return count;
}
	
static DEVICE_ATTR(disable, 0664, uc6511_disable_show, uc6511_disable_store);

static struct attribute *uc6511_attributes[] = {
	&dev_attr_disable.attr,
	&dev_attr_pen_down.attr,
	NULL
};

static const struct attribute_group uc6511_attr_group = {
	.attrs = uc6511_attributes,
};

static int __devinit uc6511_construct(bus_device *bus, struct uc6511 *ts)
{
	struct input_dev *input_dev;
	struct uc6511_platform_data *pdata = bus->dev.platform_data;
	int err;
	u16 revid;
	u32  tmp;
/*
	tmp = __raw_readl(rEINTCON);    
	tmp &= ~(0x7<<4);
	tmp |= (0x2<<4);// FALL EDGE   
	__raw_writel(tmp,rEINTCON);      
	                                
	tmp = __raw_readl(rEINTFLTCON0);  
	tmp &= ~(0xff<<8);              
	tmp |= ((0x1<<8) | (0x7f <<8));   
	__raw_writel(tmp,rEINTFLTCON0);
*/
	printk(KERN_INFO "uc6511_construct +\n");
	if (!bus->irq) {
		dev_err(&bus->dev, "no IRQ?\n");
		return -ENODEV;
	}
#if 0
	if (!pdata) {
		dev_err(&bus->dev, "no platform data?\n");
		return -ENODEV;
	}
#endif
	input_dev = input_allocate_device();
	if (!input_dev)
		return -ENOMEM;

	ts->input = input_dev;

	INIT_WORK(&ts->work, uc6511_work);
//	INIT_DELAYED_WORK(&(ts->dwork), uc6511_work);
	mutex_init(&ts->mutex);
//	setup_timer(&irq_timer, uc6511_timer_func, 0);
	hrtimer_init(&ts_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts_timer.function = uc6511_timer_func;


	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(&bus->dev));

	input_dev->name = "UC6511 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->dev.parent = &bus->dev;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(ABS_X, input_dev->absbit);
	__set_bit(ABS_Y, input_dev->absbit);
	__set_bit(ABS_PRESSURE, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);

	input_set_abs_params(input_dev, ABS_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	//err = uc6511_write(bus, AD7879_REG_CTRL2, AD7879_RESET);

	if (err < 0) {
		dev_err(&bus->dev, "Failed to write %s\n", input_dev->name);
		goto err_free_mem;
	}

	err = request_irq(bus->irq, uc6511_irq,
			  IRQF_DISABLED, bus->dev.driver->name, ts);

	if (err) {
		dev_err(&bus->dev, "irq %d busy?\n", bus->irq);
		goto err_free_mem;
	}

	err = sysfs_create_group(&bus->dev.kobj, &uc6511_attr_group);
	if (err)
		goto err_free_irq;

	err = input_register_device(input_dev);
	if (err)
		goto err_remove_attr;

	dev_info(&bus->dev, "Rev.%d touchscreen, irq %d\n",
		 revid >> 8, bus->irq);

	printk(KERN_INFO "uc6511_construct -\n");
	return 0;

err_remove_attr:
	sysfs_remove_group(&bus->dev.kobj, &uc6511_attr_group);
err_free_irq:
	free_irq(bus->irq, ts);
err_free_mem:
	input_free_device(input_dev);

	printk(KERN_ERR "uc6511_construct error\n");
	return err;
}

static int __devexit uc6511_destroy(bus_device *bus, struct uc6511 *ts)
{
	uc6511_disable(ts);
	sysfs_remove_group(&ts->bus->dev.kobj, &uc6511_attr_group);
	free_irq(ts->bus->irq, ts);
	input_unregister_device(ts->input);
	dev_dbg(&bus->dev, "unregistered touchscreen\n");

	return 0;
}

#if 0	//CONFIG_PM
static int uc6511_suspend(bus_device *bus, pm_message_t message)
{
	struct uc6511 *ts = dev_get_drvdata(&bus->dev);

	uc6511_disable(ts);

	return 0;
}

static int uc6511_resume(bus_device *bus)
{
	struct uc6511 *ts = dev_get_drvdata(&bus->dev);

	uc6511_enable(ts);

	return 0;
}
#else
#define uc6511_suspend NULL
#define uc6511_resume  NULL
#endif

/* All registers are word-sized.
 * AD7879 uses a high-byte first convention.
 */

static int __devinit uc6511_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct uc6511 *ts;
	int error;

	ts = kzalloc(sizeof(struct uc6511), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	uc_ts = ts;
	i2c_set_clientdata(client, ts);
	ts->bus = client;

	error = uc6511_construct(client, ts);
	if (error) {
		i2c_set_clientdata(client, NULL);
		kfree(ts);
	}

	return error;
}

static int __devexit uc6511_remove(struct i2c_client *client)
{
	struct uc6511 *ts = dev_get_drvdata(&client->dev);

	uc6511_destroy(client, ts);
	i2c_set_clientdata(client, NULL);
	kfree(ts);

	return 0;
}

static const struct i2c_device_id uc6511_id[] = {
	{ "uc6511", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, uc6511_id);

static struct i2c_driver uc6511_driver = {
	.driver = {
		.name	= "uc6511",
		.owner	= THIS_MODULE,
	},
	.id_table	= uc6511_id,
	.probe		= uc6511_probe,
	.remove		= __devexit_p(uc6511_remove),
	.suspend	= uc6511_suspend,
	.resume		= uc6511_resume,
};

static int __init uc6511_init(void)
{
	printk(KERN_INFO "uc6511_init +\n");

	struct i2c_board_info info;
	struct i2c_adapter *adapter;
	struct i2c_client *client;

	memset(&info, 0, sizeof(struct i2c_board_info));

	uc6511_int  = __imapx_name_to_gpio(CONFIG_TP_UC6511_INT);
	if(uc6511_int == IMAPX_GPIO_ERROR) {
		printk(KERN_ERR "failed to get uc6511_int pin.\n");
		return -1;
	}
	uc6511_irq_no = imapx_gpio_to_irq(uc6511_int);
	imapx_gpio_setcfg(uc6511_int, IG_INPUT, IG_NORMAL);
	imapx_gpio_setirq(uc6511_int, FILTER_MAX, IG_FALL, 1);

	info.irq = uc6511_irq_no;
	info.addr = 0x0a;
	strlcpy(info.type, "uc6511", I2C_NAME_SIZE);

	adapter = i2c_get_adapter(CONFIG_TP_UC6511_I2C + 1);
	if (!adapter) {
		printk("*******get_adapter error!\n");
	}
	client = i2c_new_device(adapter, &info);

	printk(KERN_INFO "uc6511_init +\n");

	return i2c_add_driver(&uc6511_driver);
}
module_init(uc6511_init);

static void __exit uc6511_exit(void)
{
	i2c_del_driver(&uc6511_driver);
	printk(KERN_INFO "uc6511_exit +\n");
}
module_exit(uc6511_exit);

MODULE_AUTHOR("Lex Yang <lex.yang@taiwan-sf.com>");
MODULE_DESCRIPTION("UC6511(-1) touchscreen Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:uc6511");
