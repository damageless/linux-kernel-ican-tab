/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * 
 * re-write by warits. apr.25.2011
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <asm/io.h>
#include <asm-generic/bitops/non-atomic.h>
#include <asm/gpio.h>
#include <mach/imapx_gpio.h>
#include <mach/irqs.h>
#include <mach/imapx_intr.h>
#include <mach/imapx_sysmgr.h>

/* define this if your hp inject have dithering
 * unit in ms
 */
#define HP_DITHERING_FILTER		500

//#define DEBUG
#ifdef DEBUG
#define ix_keys_dbg(x...) printk(KERN_ERR "ix-keys: " x)
#else
#define ix_keys_dbg(x...)
#endif

#ifdef CONFIG_IG_DEVICE_HDMI
/*disable sleep mode if using HDMI*/
extern unsigned int VIDEO_MODE;
extern unsigned int DOUBLE_DISPLAY_MODE;
extern unsigned int PANAUISION_MODE;
#endif

/* global key structure */
struct ix_key {
	char		name[16];
	char		spin[8];
	int			code;
	unsigned int	gpio;
	int			irq;
	int			stat;
	struct work_struct work;
	struct input_dev *input;
};

struct ix_key ix_keys[] = {
	{
		.name = "sleep",
		.spin = "N",
		.code = KEY_SLEEP,
	}, {
		.name = "power",
		.spin = "N",
		.code = KEY_POWER,
	},
#ifdef CONFIG_IG_KEYS_POWERS
	{
		.name = "menu",
		.spin = CONFIG_IG_KEYS_MENU,
		.code = KEY_MENU,
	}, {
		.name = "back",
		.spin = CONFIG_IG_KEYS_BACK,
		.code = KEY_BACK,
	}, {
		.name = "home",
		.spin = CONFIG_IG_KEYS_HOME,
		.code = KEY_HOME,
	}, {
		.name = "search",
		.spin = CONFIG_IG_KEYS_SEARCH,
		.code = KEY_SEARCH,
	}, {
		.name = "volumeup",
		.spin = CONFIG_IG_KEYS_VOLUP,
		.code = KEY_VOLUMEUP,
	}, {
		.name = "volumedown",
		.spin = CONFIG_IG_KEYS_VOLDOWN,
		.code = KEY_VOLUMEDOWN,
	},
#endif
#ifdef CONFIG_IG_AUDIO_POWERS
	{
		.name = "hp",
		.spin = CONFIG_IG_AUDIO_HPIN,
		.code = KEY_PHONE,
	},
#endif
#ifdef CONFIG_IG_KEYS_LEDS
	{
		.name = "led0",
		.spin = CONFIG_IG_KEYS_LED0,
		.code = 0,
	},
#endif
#ifdef CONFIG_IG_MOTOR_POWERS
	{
		.name = "motor",
		.spin = CONFIG_IG_MOTOR_ENABLE,
		.code = 0,
	}
#endif
};


/* small API */
static void ix_keys_gpio_updown(struct work_struct *work)
{
	struct ix_key *key = container_of(work, struct ix_key, work);

	imapx_gpio_setpin(key->gpio, 1, IG_NORMAL);
	msleep(key->stat);
	imapx_gpio_setpin(key->gpio, 0, IG_NORMAL);
}

int imap_iokey_led(int en, int ms)
{
	static struct ix_key *key = NULL;
	int i;

	if(!key) {
		for(i = 0; i < ARRAY_SIZE(ix_keys); i++)
		  if(!strncmp(ix_keys[i].name, "led0", 4)) {
			  key = &ix_keys[i];
			  break;
		  }
	}

	if(!key) {
		printk(KERN_ERR "led0 is not defined.\n");
		return -1;
	}
	key->stat = ms;
	schedule_work(&key->work);
	return 0;
}

int imap_iokey_motor(int en, int ms)
{
	static struct ix_key *key = NULL;
	int i;

	if(!key) {
		for(i = 0; i < ARRAY_SIZE(ix_keys); i++)
		  if(!strncmp(ix_keys[i].name, "motor", 5)) {
			  key = &ix_keys[i];
			  break;
		  }
	}

	if(!key) {
		printk(KERN_ERR "led0 is not defined.\n");
		return -1;
	}
	key->stat = ms;
	schedule_work(&key->work);
	return 0;
}

int imap_iokey_emu(unsigned char c)
{
	input_event(ix_keys[0].input, EV_KEY, c, 1);
	input_event(ix_keys[0].input, EV_KEY, c, 0);

	ix_keys_dbg("Emulating KEY: %d\n", c);
	return 0;
}

#ifdef CONFIG_IG_AUDIO_POWERS
static struct timer_list spk_timer;
static unsigned long phonelt = 0;

int imap_iokey_spken(int en)
{
	uint32_t spk, hp;

	spk = __imapx_name_to_gpio(CONFIG_IG_AUDIO_SPKEN);
	if(spk == IMAPX_GPIO_ERROR) {
		printk(KERN_ERR "failed to get spken pin.\n");
		return -1;
	}

	hp = __imapx_name_to_gpio(CONFIG_IG_AUDIO_HPIN);
	if(hp == IMAPX_GPIO_ERROR) {
		printk(KERN_ERR "failed to get hp pin.\n");
		return -1;
	}

	imapx_gpio_getpin(hp, IG_NORMAL)?: (en = 0);
	imapx_gpio_setcfg(spk, IG_OUTPUT, IG_NORMAL);
	imapx_gpio_setpin(spk, !!en, IG_NORMAL);
	return 0;
}

void ix_keys_spk_delayed_en(unsigned long x)
{
	if(jiffies > (phonelt
		   + (HP_DITHERING_FILTER * HZ) / 1000 - 1))
	  imap_iokey_spken(1);
}
EXPORT_SYMBOL(imap_iokey_spken);
#endif
EXPORT_SYMBOL(imap_iokey_led);
EXPORT_SYMBOL(imap_iokey_motor);
EXPORT_SYMBOL(imap_iokey_emu);

static void ix_keys_work(struct work_struct *work)
{
	struct ix_key *key = container_of(work, struct ix_key, work);
	static unsigned long last = 0;

	if(unlikely(key->code == KEY_PHONE))
	{
		phonelt = jiffies;
		/* head phone status changed */
#ifdef CONFIG_IG_AUDIO_POWERS
		mod_timer(&spk_timer,
		   jiffies + msecs_to_jiffies(HP_DITHERING_FILTER));
#endif
	} else if (unlikely(key->code == KEY_SLEEP)) {
		if(readl(rPOW_STB) & 0x1) {
			writel(0x1, rPOW_STB);
			if(likely(jiffies > last + HZ * 4))
			{
				input_event(key->input, EV_KEY, CONFIG_IG_POWER_KEY_EMU, 1);
				input_event(key->input, EV_KEY, CONFIG_IG_POWER_KEY_EMU, 0);
				ix_keys_dbg("Emulating KEY: %d\n", CONFIG_IG_POWER_KEY_EMU);
			}
		}
		msleep(10);
		enable_irq(IRQ_PowerMode);
	} else if (unlikely(key->code == KEY_POWER)) {
		if(readl(rPOW_STB) & 0x2) {
			input_event(key->input, EV_KEY, KEY_POWER, 0);
			input_event(key->input, EV_KEY, KEY_POWER, 1);
			ix_keys_dbg("Reporting KEY: POWER\n");

			while(readl(rPOW_STB) & 0x2) {
				writel(0x2, rPOW_STB);
				msleep(10);
				last = jiffies;
			}
		}
		msleep(10);
		enable_irq(IRQ_PowerMode);
	} else {
		input_event(key->input, EV_KEY, key->code,
		   !imapx_gpio_getpin(key->gpio, IG_NORMAL));

		ix_keys_dbg("Reporting KEY: %d.%d\n", key->code,
		   !imapx_gpio_getpin(key->gpio, IG_NORMAL));

	}
}

static irqreturn_t ix_keys_isr(int irq, void *dev_id)
{
	struct ix_key *key = dev_id;

	ix_keys_dbg("%s invoked with name %s\n", __func__, key->name);

	if((key->code == KEY_SLEEP) ||
		(key->code == KEY_POWER))
	{
		/* ignore the pow_rnd interrupt */
		if(readl(rPOW_STB) & 0x4)
		  writel(0x4, rPOW_STB);

		/* this is a classic message,
		 * you will be panic is it disappears
		 */
		ix_keys_dbg("tmp is %d\n", readl(rPOW_STB));
#ifdef CONFIG_IG_DEVICE_HDMI
		if(!((VIDEO_MODE == 1) || (DOUBLE_DISPLAY_MODE == 1) || (PANAUISION_MODE == 1)))
		{
#endif
			if(readl(rPOW_STB) & 0x3){      //in case of irq loop
				disable_irq_nosync(IRQ_PowerMode);
				schedule_work(&key->work);
			}
#ifdef CONFIG_IG_DEVICE_HDMI
		}else{
			if(readl(rPOW_STB) & 0x7)
				writel(readl(rPOW_STB), rPOW_STB);
		}
#endif
		} else {
			/* this is a normal key */
			if(unlikely(imapx_gpio_is_pending(key->gpio, 1)))
		   schedule_work(&key->work);
	}

	return IRQ_HANDLED;
}


static int __devinit ix_keys_probe(struct platform_device *pdev)
{
	struct input_dev *input;
	int i, error;

#ifdef CONFIG_IG_AUDIO_POWERS
	setup_timer(&spk_timer, ix_keys_spk_delayed_en, 0);
	imap_iokey_spken(1);
#endif

	input = input_allocate_device();
	if (!input) {
		printk(KERN_ERR "allocate input device failed.\n");
		return -ENOMEM;
	}

	for(i = 0; i < ARRAY_SIZE(ix_keys); i++)
	{
		if(unlikely(!ix_keys[i].code)) {
			/* such IO like led & motor, nothing like a key */
			ix_keys[i].gpio	= __imapx_name_to_gpio(ix_keys[i].spin);
			
			imapx_gpio_setcfg(ix_keys[i].gpio, IG_OUTPUT, IG_NORMAL);
			INIT_WORK(&ix_keys[i].work, ix_keys_gpio_updown);
			continue;
		}
		else if(unlikely((ix_keys[i].code == KEY_POWER) ||
			   (ix_keys[i].code == KEY_SLEEP))) {
			INIT_WORK(&ix_keys[i].work, ix_keys_work);
			ix_keys[i].irq = IRQ_PowerMode;
		}
		else {
			INIT_WORK(&ix_keys[i].work, ix_keys_work);
			ix_keys[i].gpio	= __imapx_name_to_gpio(ix_keys[i].spin);

			if(ix_keys[i].gpio == IMAPX_GPIO_ERROR)
			  /* this gpio value is not defined */
			  continue;

			ix_keys[i].irq	= imapx_gpio_to_irq(ix_keys[i].gpio);
			imapx_gpio_setcfg(ix_keys[i].gpio, IG_INPUT, IG_NORMAL);
			/* configurate the pins */
			imapx_gpio_setirq(ix_keys[i].gpio, FILTER_MAX, IG_BOTH, 1);
		}

		ix_keys[i].input = input;
		input_set_capability(input, EV_KEY, ix_keys[i].code);

		error = request_irq(ix_keys[i].irq, ix_keys_isr,
		   IRQF_DISABLED | IRQF_SHARED, ix_keys[i].name, &ix_keys[i]);

		if(error) {
			printk(KERN_ERR "claim irq %d failed, name %s\n",
			   ix_keys[i].irq, ix_keys[i].name);
			goto __fail_exit__;
		} else
		  printk(KERN_ERR "claim irq %d OK, name %s\n", ix_keys[i].irq,
			 ix_keys[i].name);
	}

	/* add the emulate bit */
	input_set_capability(input, EV_KEY, CONFIG_IG_POWER_KEY_EMU);

	input->name = "gpio-keys";
	input->phys = "gpio-keys/input0";
	input->dev.parent = &pdev->dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	error = input_register_device(input);
	if (error) {
		pr_err("gpio-keys: Unable to register input device, "
				"error: %d\n", error);
		goto __fail_exit__;
	}

	/* light the led */
	imap_iokey_led(1, 2000);

	return 0;

__fail_exit__:
	input_free_device(input);

	return error;
}

static int __devexit ix_keys_remove(struct platform_device *pdev)
{
	input_unregister_device(ix_keys->input);
	return 0;
}


#ifdef CONFIG_PM
static int
ix_keys_suspend(struct platform_device *pdev, pm_message_t state) {
	return 0;
}

static int
ix_keys_resume(struct platform_device *pdev) {
	return 0;
}
#else
#define ix_keys_suspend	NULL
#define ix_keys_resume	NULL
#endif

static struct platform_driver ix_keys_device_driver = {
	.probe		= ix_keys_probe,
	.remove		= __devexit_p(ix_keys_remove),
	.suspend	= ix_keys_suspend,
	.resume		= ix_keys_resume,
	.driver		= {
		.name	= "gpio-keys",
		.owner	= THIS_MODULE,
	}
};

static int __init ix_keys_init(void)
{
	return platform_driver_register(&ix_keys_device_driver);
}

static void __exit ix_keys_exit(void)
{
	platform_driver_unregister(&ix_keys_device_driver);
}

module_init(ix_keys_init);
module_exit(ix_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Keyboard driver for CPU GPIOs");
MODULE_ALIAS("platform:gpio-keys");
