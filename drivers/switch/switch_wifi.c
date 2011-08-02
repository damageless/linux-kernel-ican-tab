#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <plat/imapx.h>

#ifdef CONFIG_IG_WIFI_SDIO
#define __mmc_rescan(x) sdhci_mmc##x##_detect_change()
#define _mmc_rescan(x) __mmc_rescan(x)
extern void	_mmc_rescan(CONFIG_IG_WIFI_SDIO_CHANNEL);
#endif

struct wifi_switch_data {
	struct switch_dev sdev;
	unsigned gpio_power0;
	unsigned gpio_power1;
	unsigned gpio_switch;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	//struct work_struct *work;
};

static struct wifi_switch_data *switch_data;

static struct work_struct switch_wifi_work;

void wifi_power(int power)
{
	if (power == 1)
	{
		imapx_gpio_setpin(switch_data->gpio_power0, 1, IG_NMSL);
		imapx_gpio_setpin(switch_data->gpio_power1, 1, IG_NMSL);

#ifdef CONFIG_IG_WIFI_SDIO
		msleep(1000);
		_mmc_rescan(CONFIG_IG_WIFI_SDIO_CHANNEL);
#endif
		msleep(1500);
	}
	else {
		imapx_gpio_setpin(switch_data->gpio_power0, 0, IG_NMSL);
		imapx_gpio_setpin(switch_data->gpio_power1, 0, IG_NMSL);
	}
}
EXPORT_SYMBOL(wifi_power);

static void gpio_switch_work(struct work_struct *work)
{
//	printk("enter function %s, at line %d \n", __func__, __LINE__);

	switch_set_state(&switch_data->sdev,
	   imapx_gpio_getpin(switch_data->gpio_switch, IG_NORMAL));
}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
//	printk("enter function %s, at line %d \n", __func__, __LINE__);
	if(imapx_gpio_is_pending(switch_data->gpio_switch, 1))
	  schedule_work(&switch_wifi_work);

	return IRQ_HANDLED;
}

static ssize_t switch_wifi_print_state(struct switch_dev *sdev, char *buf)
{
	const char *state;
	//printk("enter function %s, at line %d \n", __func__, __LINE__);
	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);

	return -1;
}

static int wifi_switch_probe(struct platform_device *pdev)
{
	struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;
//	struct wifi_switch_data *switch_data;
	int ret;

	//printk("enter function %s, at line %d \n", __func__, __LINE__);

	if (!pdata)
		return -EBUSY;

	switch_data = kzalloc(sizeof(struct wifi_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->gpio_power0 = __imapx_name_to_gpio(CONFIG_IG_WIFI_POWER0);
	switch_data->gpio_power1 = __imapx_name_to_gpio(CONFIG_IG_WIFI_POWER1);

#ifdef CONFIG_IG_KEYS_POWERS
	switch_data->gpio_switch = __imapx_name_to_gpio(CONFIG_IG_KEYS_WIFI);
#else
	switch_data->gpio_switch = IMAPX_GPIO_ERROR;
#endif

	imapx_gpio_setcfg(switch_data->gpio_power0, IG_OUTPUT, IG_NMSL);
	imapx_gpio_setcfg(switch_data->gpio_power1, IG_OUTPUT, IG_NMSL);

	wifi_power(0);

	if(switch_data->gpio_switch == IMAPX_GPIO_ERROR)
	{
		printk(KERN_ERR "get wifi powers/switch pins failed.\n");
		return -ENOTTY;
	}

	switch_data->sdev.name = pdata->name;
//	switch_data->gpio = pdata->gpio;
	switch_data->name_on = pdata->name_on;
	switch_data->name_off = pdata->name_off;
	switch_data->state_on = pdata->state_on;
	switch_data->state_off = pdata->state_off;
	switch_data->sdev.print_state = switch_wifi_print_state;

//	printk("switch_gpio is %d\n", switch_data->gpio);

	INIT_WORK(&switch_wifi_work, gpio_switch_work);

	switch_data->irq = imapx_gpio_to_irq(switch_data->gpio_switch);
	ret = request_irq(switch_data->irq, gpio_irq_handler,
	   IRQF_DISABLED, pdev->name, switch_data);

	if (ret < 0)
	  return ret;

    ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
	  return ret;

	imapx_gpio_setcfg(switch_data->gpio_switch, IG_INPUT, IG_NORMAL);
	/* Perform initial detection */
	schedule_work(&switch_wifi_work);
//	gpio_switch_work(&switch_wifi_work);
	imapx_gpio_setirq(switch_data->gpio_switch, FILTER_MAX, IG_BOTH, 1);

//	printk("enter function %s, at line %d \n", __func__, __LINE__);
	return 0;
}

static int __devexit wifi_switch_remove(struct platform_device *pdev)
{
	struct wifi_switch_data *switch_data = platform_get_drvdata(pdev);

	cancel_work_sync(&switch_wifi_work);
    switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);
	return 0;
}

static int wifi_switch_suspend(struct platform_device *pdev, pm_message_t state)
{
	//struct wifi_switch_data *switch_data = platform_get_drvdata(pdev);

	//schedule_work(&switch_wifi_work);
	return 0;
}

static int wifi_switch_resume(struct platform_device *pdev)
{
	//struct wifi_switch_data *switch_data = platform_get_drvdata(pdev);

	schedule_work(&switch_wifi_work);
	return 0;
}

static struct platform_driver wifi_switch_driver = {
	.probe		= wifi_switch_probe,
	.remove		= __devexit_p(wifi_switch_remove),
	.suspend	= wifi_switch_suspend,
	.resume		= wifi_switch_resume,
	.driver		= {
		.name	= "switch-wifi",
		.owner	= THIS_MODULE,
	},
};

static int __init wifi_switch_init(void)
{
	printk("wifi_switch module init\n");
	return platform_driver_register(&wifi_switch_driver);
}

static void __exit wifi_switch_exit(void)
{
	platform_driver_unregister(&wifi_switch_driver);
}

module_init(wifi_switch_init);
//late_initcall(wifi_switch_init);
module_exit(wifi_switch_exit);

MODULE_AUTHOR("Bob.yang <Bob.yang@infotmic.com.cn>");
/* modified by warits on apr.28 to fit new gpio structure */
MODULE_DESCRIPTION("WIFI Switch driver");
MODULE_LICENSE("GPL");
