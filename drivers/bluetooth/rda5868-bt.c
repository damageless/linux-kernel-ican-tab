/*
 * rda5868 chip Bluetooth control
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/rfkill.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <linux/tty.h>

#include <asm/io.h>
#include <plat/imapx.h>

#include "rda5868-bt.h"

#ifdef BTPM_DEBUG_PRINTK
#define btpm_dbg_msg(fmt, arg...) printk(KERN_DEBUG "BTPM: %s: " fmt "\n", __FUNCTION__ , ## arg)
#else
#define btpm_dbg_msg(fmt, arg...)
#endif
#define btpm_err_msg(fmt, arg...) printk(KERN_ERR "BTPM: %s: " fmt "\n", __FUNCTION__ , ## arg)
#define btpm_info_msg(fmt, arg...) printk(KERN_INFO "BTPM: %s: " fmt "\n", __FUNCTION__ , ## arg)

static int rda5868_set_bt_power(int on)
{
	int tmp;
	//power on or off
	if (on)
	{
		btpm_dbg_msg("set BT_LDOON\n");
		tmp = readl(rGPODAT);
		tmp |= 0x1 << BT_LDOON;
		writel(tmp, rGPODAT);

		btpm_dbg_msg("set BT reset low 1\n");
		tmp = readl(rGPODAT);
		tmp &= ~(0x1 << BT_RESETN);
		writel(tmp, rGPODAT);

		btpm_dbg_msg("wait for 26M XTAL\n");
		msleep(20);

		btpm_dbg_msg("set BT reset high 1\n");
		tmp = readl(rGPODAT);
		tmp |= 0x1 << BT_RESETN;
		writel(tmp, rGPODAT);

		//configure the 5868 RF via I2C
		msleep(1);

		//set RESET to low to reset RDA5868 core
		btpm_dbg_msg("set BT reset low 2\n");
		tmp = readl(rGPODAT);
		tmp &= ~(0x1 << BT_RESETN);
		writel(tmp, rGPODAT);

		msleep(1);

		btpm_dbg_msg("set BT reset high 2\n");
		tmp = readl(rGPODAT);
		tmp |= 0x1 << BT_RESETN;
		writel(tmp, rGPODAT);
		//the uart can be used after 20ms
		msleep(250);
	}
	else
	{
		btpm_dbg_msg("set BT_RESET_LOW 3\n");
		tmp = readl(rGPODAT);
		tmp &= ~(0x1 << BT_RESETN);
		writel(tmp, rGPODAT);

		btpm_dbg_msg("set BT_LDO_OFF 2\n");
		tmp = readl(rGPODAT);
		tmp &= ~(0x1 << BT_LDOON);
		writel(tmp, rGPODAT);
	}

	return 0;
}

static int rda5868_bt_set_block(void *data, bool blocked)
{
	if (!blocked)
	{
		rda5868_set_bt_power(1);
	}
	else
	{
		rda5868_set_bt_power(0);
	}

	return 0;
}

static const struct rfkill_ops rda5868_bt_rfkill_ops = {
	.set_block = rda5868_bt_set_block,
};

static int rda5868_bt_probe(struct platform_device *dev)
{
	int rc;
	struct rfkill *rfk;
	void *data = (void *)dev;
	uint32_t tmp;

	btpm_dbg_msg(">> RDA5868 >>rda5868_bt_probe 01\n");
	//printk("enter function %s, at line %d \n", __func__, __LINE__);
	tmp = readl(rGPOCON);
	tmp &= ~(0xf<<24);
	tmp |= 0x5<<24;

	rfk = rfkill_alloc("rda5868-bt", &dev->dev, RFKILL_TYPE_BLUETOOTH, &rda5868_bt_rfkill_ops, data);
	if (!rfk)
	{
		rc = -ENOMEM;
		goto err_rfk_alloc;
	}

	btpm_dbg_msg(" >> RDA5868 >>rda5868_bt_probe 02\n");
	rc = rfkill_register(rfk);
	if (rc)
	{
		goto err_rfkill;
	}

	platform_set_drvdata(dev, rfk);

	rda5868_set_bt_power(0);
	btpm_dbg_msg(" >> RDA5868 >>rda5868_bt_probe 03\n");
	return 0;

err_rfkill:
	if (rfk)
	  rfkill_destroy(rfk);
	rfk = NULL;

err_rfk_alloc:
	rda5868_set_bt_power(0);
	return rc;

}

static int __devexit rda5868_bt_remove(struct platform_device *dev)
{
	struct rfkill *rfk = platform_get_drvdata(dev);

	if (rfk)
	  rfkill_unregister(rfk);
	rfk = NULL;

	rda5868_set_bt_power(0);

	return 0;
}

static int rda5868_bt_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int rda5868_bt_resume(struct platform_device *dev)
{
	return 0;
}

static struct platform_driver rda5868_bt_driver = {
	.probe = rda5868_bt_probe,
	.remove = __devexit_p(rda5868_bt_remove),
	.suspend = rda5868_bt_suspend,
	.resume = rda5868_bt_resume,
	.driver = {
		.name = "rda5868-bt",
		.owner = THIS_MODULE,
	},
};

static int __init rda5868_bt_init(void)
{
	printk("RDA5868 module init");
	return platform_driver_register(&rda5868_bt_driver);
}

static void __exit rda5868_bt_exit(void)
{
	platform_driver_unregister(&rda5868_bt_driver);
}

MODULE_AUTHOR("bob.yang@infotmic.com.cn");
MODULE_DESCRIPTION("RDA 5868 Bluetooth power management via RFKILL.");
MODULE_LICENSE("GPL");

//module_init(rda5868_bt_init);
late_initcall(rda5868_bt_init);
module_exit(rda5868_bt_exit);
