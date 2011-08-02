/*
 * * Fake Battery driver for android
 * *
 * * Copyright © 2009 Rockie Cheng <aokikyon@gmail.com>
 * *
 * * This program is free software; you can redistribute it and/or modify
 * * it under the terms of the GNU General Public License version 2 as
 * * published by the Free Software Foundation.
 * */
/*
 * the condition that low level voltage warning message can display are:
 * 1, the voltage lower than 15%
 * 2, there is a debounce between two voltage and condition 1 is satisfied.
 **/
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <mach/imapx_base_reg.h>
#include <linux/io.h>
#include <asm/delay.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <mach/imapx_gpio.h>
#include <linux/gpio.h>

#if (defined(CONFIG_TP_TI2046) || defined(CONFIG_BATT_TI2046))
#include <linux/spi/ads7846.h>
#endif
#if defined(CONFIG_BATT_TLV0832)
#include "tlv0832.h"
#endif

#define BAT_STAT_PRESENT 0x01
#define BAT_STAT_FULL   0x02
#define BAT_STAT_LOW   0x04
#define BAT_STAT_DESTROY 0x08
#define BAT_STAT_AC   0x10
#define BAT_STAT_CHARGING 0x20
#define BAT_STAT_DISCHARGING 0x40

#define BAT_ERR_INFOFAIL 0x02
#define BAT_ERR_OVERVOLTAGE 0x04
#define BAT_ERR_OVERTEMP 0x05
#define BAT_ERR_GAUGESTOP 0x06
#define BAT_ERR_OUT_OF_CONTROL 0x07
#define BAT_ERR_ID_FAIL   0x09
#define BAT_ERR_ACR_FAIL 0x10

#define BAT_ADDR_MFR_TYPE 0x5F
#define IMAP_ADAPTER 615
#define IMAP_BAT_FULL 1500
#define IMAP_BAT_CRITICAL 1000
#define BATT_INITIAL 0
#define BATT_ON 1
#define BATT_AGAIN 2

//int batt_init_ok=0;
//export_symbol(batt_init_ok);

struct imap_batt_data_list {
	unsigned int	capacity;
	unsigned int	charging_val;
	unsigned int	charging_count;
	unsigned int	uncharging_val;
	unsigned int	uncharging_count;
};

#ifdef CONFIG_BATT_REGION_SET
struct imap_batt_data_list imap_batt_datas[] = {	// battery datas must be alligned in increase order of capacitty values, 
	{0,  CONFIG_BATT_CHG_LEV0,  48, CONFIG_BATT_LEV0,  45},			// and the first must be zero, the last must be 100;
	{10, CONFIG_BATT_CHG_LEV10, 48, CONFIG_BATT_LEV10, 45},
	{20, CONFIG_BATT_CHG_LEV20, 48, CONFIG_BATT_LEV20, 45},
	{30, CONFIG_BATT_CHG_LEV30, 48, CONFIG_BATT_LEV30, 45},
	{40, CONFIG_BATT_CHG_LEV40, 48, CONFIG_BATT_LEV40, 45},
	{50, CONFIG_BATT_CHG_LEV50, 48, CONFIG_BATT_LEV50, 45},
	{60, CONFIG_BATT_CHG_LEV60, 48, CONFIG_BATT_LEV60, 45},
	{70, CONFIG_BATT_CHG_LEV70, 48, CONFIG_BATT_LEV70, 45},
	{80, CONFIG_BATT_CHG_LEV80, 48, CONFIG_BATT_LEV80, 45},
	{90, CONFIG_BATT_CHG_LEV90, 48, CONFIG_BATT_LEV90, 45},
	{100,CONFIG_BATT_CHG_LEV100,48, CONFIG_BATT_LEV100,45},
};
#else
struct imap_batt_data_list imap_batt_datas[] = {	// battery datas must be alligned in increase order of capacitty values, 
	{0,  450, 48, 420, 45},			// and the first must be zero, the last must be 100;
	{10, 468, 48, 438, 45},
	{20, 486, 48, 456, 45},
	{30, 496, 48, 466, 45},
	{40, 508, 48, 478, 45},
	{50, 516, 48, 486, 45},
	{60, 524, 48, 494, 45},
	{70, 536, 48, 506, 45},
	{80, 546, 48, 516, 45},
	{90, 561, 48, 531, 45},
	{100,585, 48, 555, 45},
};
#endif
static int battery_state = POWER_SUPPLY_STATUS_UNKNOWN;
static int battery_old_state = POWER_SUPPLY_STATUS_UNKNOWN;
static struct timer_list supply_timer;
static int batt;
static int capacity_show = 120, capacity_old_show = 0;
static int count = 0;
volatile static int adaptor_disconnect_num=0;
volatile static int charging_status = 0;

unsigned int chg_ful;
unsigned int adp_in;

int get_adc_default_val(void)
{
	return 0;
}
int (*get_adc_val)(void) = get_adc_default_val;

void __imapx_register_batt(int (*func)(void))
{
	 get_adc_val = func;
}

EXPORT_SYMBOL(__imapx_register_batt);

static int ischargingfull(void)
{
	unsigned long tmp;
	tmp = imapx_gpio_getpin(chg_ful,IG_NORMAL);
	//printk("ischaringfull is %d\n",tmp);
        //printk("batt is %d\n",batt);
	return tmp; 
}

static int isacon(void)
{
	unsigned long tmp;
	tmp = imapx_gpio_getpin(adp_in,IG_NORMAL);
  	//printk("isacon is %d\n",tmp);
	return tmp; 
}

static int batt2capacity(int batt_value)
{
	unsigned int capacity_value = 0;
	int i;
	int n = sizeof(imap_batt_datas) / sizeof(unsigned int) / 5;

	if (ischargingfull())
		capacity_value = 100;
	else if (isacon())
	{
		if (batt_value <= imap_batt_datas[0].charging_val)
			capacity_value = 0;
		else
		{
			for (i=1; i<n; i++)
			{
				if (batt_value <= imap_batt_datas[i].charging_val)
				{
					capacity_value = 
						( batt_value - imap_batt_datas[i-1].charging_val )
						* ( imap_batt_datas[i].capacity - imap_batt_datas[i-1].capacity )
						 /( imap_batt_datas[i].charging_val - imap_batt_datas[i-1].charging_val)
						 + imap_batt_datas[i-1].capacity;
					break;
				}
			}
			if (batt_value > imap_batt_datas[n-1].charging_val)
				capacity_value = 100;
		}
	//	printk("now is charging, batt_value: %d, imap_batt_datas: %d,%d,%d,%d,%d, capacity_value: %d\n",
	//		       batt_value, imap_batt_datas[i].capacity,imap_batt_datas[i].charging_val,imap_batt_datas[i].charging_count,imap_batt_datas[i].uncharging_val,imap_batt_datas[i].uncharging_count,capacity_value);	
	}
	else
	{
		if (batt_value <= imap_batt_datas[0].uncharging_val)
			capacity_value = 0;
		else
		{
			for (i=1; i<n; i++)
			{
				if (batt_value <= imap_batt_datas[i].uncharging_val)
				{
					capacity_value = 
						( batt_value - imap_batt_datas[i-1].uncharging_val )
						* ( imap_batt_datas[i].capacity - imap_batt_datas[i-1].capacity )
						 /( imap_batt_datas[i].uncharging_val - imap_batt_datas[i-1].uncharging_val)
						 + imap_batt_datas[i-1].capacity;
					break;
				}
			}
			if (batt_value > imap_batt_datas[n-1].uncharging_val)
				capacity_value = 100;
		}
	//	printk("now is uncharging, batt_value: %d, imap_batt_datas: %d,%d,%d,%d,%d, capacity_value: %d\n",
	//		       batt_value, imap_batt_datas[i].capacity,imap_batt_datas[i].charging_val,imap_batt_datas[i].charging_count,imap_batt_datas[i].uncharging_val,imap_batt_datas[i].uncharging_count,capacity_value);	
	}

	return capacity_value;
}

static int android_ac_get_prop(struct power_supply *psy,
   enum power_supply_property psp,
   union power_supply_propval *val)
{

	switch (psp)
	{
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = battery_state;
			if(val->intval > POWER_SUPPLY_STATUS_NOT_CHARGING)
			  val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		default:
			break;
	}
	return 0;
}

static enum power_supply_property android_ac_props[] =
{
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply android_ac =
{
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = android_ac_props,
	.num_properties = ARRAY_SIZE(android_ac_props),
	.get_property = android_ac_get_prop,
};

static int android_bat_get_status(union power_supply_propval *val)
{
	val->intval = battery_state;
	return 0;
}

static int android_bat_get_capacity(union power_supply_propval *val)
{
#ifdef CONFIG_BATT_DEBUG
	/* do not report zero to prevent soft shutdown */
	val->intval = (capacity_show? capacity_show: 1);
#else
	val->intval = capacity_show;
#endif

	return 0;
}


static int android_bat_get_health(union power_supply_propval *val)
{

	val->intval = POWER_SUPPLY_HEALTH_GOOD;
	return 0;
}

static int android_bat_get_mfr(union power_supply_propval *val)
{

	val->strval = "Rockie";
	return 0;
}

static int android_bat_get_tech(union power_supply_propval *val)
{
	val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
	return 0;
}

static int android_bat_get_property(struct power_supply *psy,
   enum power_supply_property psp,
   union power_supply_propval *val)
{
	int ret = 0;

	switch (psp)
	{
		case POWER_SUPPLY_PROP_STATUS:

			ret = android_bat_get_status(val);
			if (ret)
			  return ret;
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = BAT_STAT_PRESENT;
			break;

		case POWER_SUPPLY_PROP_HEALTH:
			ret = android_bat_get_health(val);
			break;
		case POWER_SUPPLY_PROP_MANUFACTURER:
			ret = android_bat_get_mfr(val);
			if (ret)
			  return ret;
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			ret = android_bat_get_tech(val);
			if (ret)
			  return ret;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_AVG:
			val->intval = 3;
			break;
		case POWER_SUPPLY_PROP_CURRENT_AVG:
			val->intval = 3;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			//val->intval = 100;
			android_bat_get_capacity(val);
			break;
		case POWER_SUPPLY_PROP_TEMP:
			val->intval = 50;
			break;
		case POWER_SUPPLY_PROP_TEMP_AMBIENT:
			val->intval = 50;
			break;
		case POWER_SUPPLY_PROP_CHARGE_COUNTER:
			val->intval = 10;
			break;
		case POWER_SUPPLY_PROP_SERIAL_NUMBER:
			val->strval = "imap210";
			break;
		default:
			ret = -EINVAL;
			break;
	}
	return ret;
}

static enum power_supply_property android_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_AMBIENT,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
};

/*********************************************************************
 * *   Initialisation
 * *********************************************************************/

static struct platform_device *bat_pdev;

static struct power_supply android_bat =
{
	.properties = android_bat_props,
	.num_properties = ARRAY_SIZE(android_bat_props),
	.get_property = android_bat_get_property,
	.use_for_apm = 1,
};

static void supply_timer_func(unsigned long unused)
{
	int capacity_val;
	int i;
	int diff_val;
	batt = get_adc_val();

#ifdef CONFIG_BATT_DEBUG
	printk(KERN_ERR "[%d] [%d] [%d] => [%d%%]\n",
				isacon(), batt, ischargingfull(), capacity_show);
#endif

	if (batt != 0)
	{
		if (count == 0)
		{
			capacity_val = batt2capacity(batt);
			if (capacity_show == 120)
				capacity_show = batt2capacity(batt);
			else if(isacon())
				capacity_show = (capacity_show+1 > 100) ? 100 : capacity_show+1;
			else if(capacity_val >= 100)
				capacity_show = capacity_show;
			else
				capacity_show = (capacity_show-1 < 0 ) ? 0 : capacity_show-1;

			diff_val = capacity_val - capacity_show;

			if (isacon())	// adaptor is connected
			{
				if (diff_val<-5)
					diff_val = -5;

				for (i=0; i<(sizeof(imap_batt_datas) / 5)-1; i++)
				{
					if (capacity_val <= imap_batt_datas[i+1].capacity)
					{
						count = imap_batt_datas[i].charging_count * 6 / (6 + diff_val);
						break;
					}
				}
				if (count == 0)
					count = imap_batt_datas[i+1].charging_count;
			}
			else	// adaptor is disconnected
			{	
				if (diff_val>5)
					diff_val = 5;

				for (i=0; i<(sizeof(imap_batt_datas) / 5)-1; i++)
				{
					if (capacity_val <= imap_batt_datas[i+1].capacity)
					{
						count = imap_batt_datas[i].uncharging_count * 6 / (6 - diff_val);
						break;
					}
				}
				if (count == 0)
					count = imap_batt_datas[i+1].charging_count;
			}
//			printk("capacity_show is %d, capacity_val is %d, and count is %d\n",capacity_show,capacity_val,count);
		}
		else
		{
			count = count - 1;
		}
	}

	battery_state = (isacon()?(ischargingfull()?
					POWER_SUPPLY_STATUS_FULL: POWER_SUPPLY_STATUS_CHARGING)
				: ((capacity_show == 100)? POWER_SUPPLY_STATUS_FULL:
					POWER_SUPPLY_STATUS_NOT_CHARGING));
	if((battery_old_state != battery_state) ||
				(capacity_old_show != capacity_show)) {
		power_supply_changed(&android_bat);
		battery_old_state = battery_state;
		capacity_old_show = capacity_show;
	}
//	time_work:
		mod_timer(&supply_timer,\
			 jiffies + msecs_to_jiffies(2000));
}
//EXPORT_SYMBOL_GPL(supply_timer_func);
static int __devinit android_bat_probe(struct platform_device *pdev)
{


	int ret = 0;
/*	
	int i;
	for(i=0;i<11;i++)
	{
	printk(KERN_EMERG"batt_val:%d....chg_val:%d\n",
			imap_batt_datas[i].charging_val, imap_batt_datas[i].uncharging_val);
	}
*/
	/**********************************************/
	//here, we need to know the power type, AC or Battery and detect the voltage each 5 second.
	//need to register a irq for pluging in or out the AC.
	//at last, need to monitor the voltage each 5 second.
	//now, we have a question, cannot display the AC icon.
	/***************************************************/
/*******************************************************/
	chg_ful = __imapx_name_to_gpio(CONFIG_BATT_CHARGE_FULL);
	if(chg_ful == IMAPX_GPIO_ERROR) {
		printk(KERN_ERR "failed to get chg_ful pin.\n");
		return -1;
	}
	adp_in = __imapx_name_to_gpio(CONFIG_BATT_ADAPTER_IN);
	if(adp_in == IMAPX_GPIO_ERROR) {
		printk(KERN_ERR "failed to get adp_in pin.\n");
		return -1;
	}
	imapx_gpio_setcfg(chg_ful, IG_INPUT, IG_NORMAL);
	imapx_gpio_pull(chg_ful,1,IG_NORMAL);
	imapx_gpio_setcfg(adp_in, IG_INPUT, IG_NORMAL);
	imapx_gpio_pull(adp_in,1,IG_NORMAL);

       setup_timer(&supply_timer, supply_timer_func, 0);
	 mod_timer(&supply_timer,\
		jiffies + msecs_to_jiffies(3000));
	/*******************************/
	ret = power_supply_register(&pdev->dev, &android_ac);
	if (ret)
	  goto ac_failed;

	android_bat.name = pdev->name;

	ret = power_supply_register(&pdev->dev, &android_bat);
	if (ret)
	  goto battery_failed;

//	else 
	//	{batt_init_ok = 1;
		 goto success;
	//	}

	power_supply_unregister(&android_bat);
battery_failed:
	power_supply_unregister(&android_ac);
ac_failed:
	platform_device_unregister(pdev);
success:
	return ret;
}

static int __devexit android_bat_remove(struct platform_device *pdev)
{
	power_supply_unregister(&android_bat);
	power_supply_unregister(&android_ac);
	platform_device_unregister(pdev);
}

#ifdef CONFIG_PM
static int
android_bat_suspend(struct platform_device *pdev, pm_message_t state) {
	
	battery_state = POWER_SUPPLY_STATUS_UNKNOWN;
	battery_old_state = POWER_SUPPLY_STATUS_UNKNOWN;
	capacity_show = 120;
	capacity_old_show = 0;
	count = 0;
	return 0;
}

static int
android_bat_resume(struct platform_device *pdev) {
	        return 0;
}
#else
#define android_bat_suspend NULL
#define android_bat_resume  NULL
#endif

static struct platform_driver android_bat_driver = {
	.probe          = android_bat_probe,
	.remove         = __devexit_p(android_bat_remove),
	.suspend        = android_bat_suspend,
	.resume         = android_bat_resume,
	.driver = {
		.name    = "battery",
		.owner   = THIS_MODULE,
	}
};

static int __init android_bat_init(void)
{
	return platform_driver_register(&android_bat_driver);
}

static void __exit android_bat_exit(void)
{
	platform_driver_unregister(&android_bat_driver);
}
module_init(android_bat_init);
module_exit(android_bat_exit);

MODULE_AUTHOR("Rockie Cheng <aokikyon@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Fake Battery driver for android");
