#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

#include <asm/io.h>
#include <plat/imapx.h>

struct sensor_switch_data {
	struct switch_dev sdev;
	unsigned gpio_switch;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	//struct work_struct *work;
};

static struct sensor_switch_data *switch_data;

static struct work_struct switch_sensor_work;

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
	  schedule_work(&switch_sensor_work);

	return IRQ_HANDLED;
}

static ssize_t switch_sensor_print_state(struct switch_dev *sdev, char *buf)
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

static int sensor_switch_probe(struct platform_device *pdev)
{
	struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;
//	struct sensor_switch_data *switch_data;
	int ret;

	//printk("enter function %s, at line %d \n", __func__, __LINE__);
	if (!pdata)
		return -EBUSY;

	switch_data = kzalloc(sizeof(struct sensor_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->gpio_switch = __imapx_name_to_gpio(CONFIG_IG_KEYS_SENSOR);

	if(switch_data->gpio_switch == IMAPX_GPIO_ERROR)
	{
		printk(KERN_ERR "get sensor powers/switch pins failed.\n");
		ret =  -ENOTTY;
		goto err_switch_dev_register;
	} else {
		imapx_gpio_setcfg(switch_data->gpio_switch, IG_INPUT, IG_NORMAL);
		imapx_gpio_setirq(switch_data->gpio_switch, FILTER_MAX, IG_BOTH, 1);
	}

	switch_data->sdev.name = pdata->name;
//	switch_data->gpio = pdata->gpio;
	switch_data->name_on = pdata->name_on;
	switch_data->name_off = pdata->name_off;
	switch_data->state_on = pdata->state_on;
	switch_data->state_off = pdata->state_off;
	switch_data->sdev.print_state = switch_sensor_print_state;

//	printk("switch_gpio is %d\n", switch_data->gpio);

	INIT_WORK(&switch_sensor_work, gpio_switch_work);

	switch_data->irq = imapx_gpio_to_irq(switch_data->gpio_switch);
	ret = request_irq(switch_data->irq, gpio_irq_handler,
	   IRQF_DISABLED, pdev->name, switch_data);

	if (ret < 0)
	  goto err_switch_dev_register;

    ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
	  goto err_switch_dev_register;

	/* Perform initial detection */
	schedule_work(&switch_sensor_work);
//	gpio_switch_work(&switch_sensor_work);

//	printk("enter function %s, at line %d \n", __func__, __LINE__);
	return 0;

err_switch_dev_register:
	kfree(switch_data);

	return ret;
}

static int __devexit sensor_switch_remove(struct platform_device *pdev)
{
	struct sensor_switch_data *switch_data = platform_get_drvdata(pdev);

	cancel_work_sync(&switch_sensor_work);
    switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);
	return 0;
}

static int sensor_switch_suspend(struct platform_device *pdev, pm_message_t state)
{
	//struct sensor_switch_data *switch_data = platform_get_drvdata(pdev);

	//schedule_work(&switch_sensor_work);
	return 0;
}

static int sensor_switch_resume(struct platform_device *pdev)
{
	//struct sensor_switch_data *switch_data = platform_get_drvdata(pdev);

	schedule_work(&switch_sensor_work);
	return 0;
}

static struct platform_driver sensor_switch_driver = {
	.probe		= sensor_switch_probe,
	.remove		= __devexit_p(sensor_switch_remove),
	.suspend	= sensor_switch_suspend,
	.resume		= sensor_switch_resume,
	.driver		= {
		.name	= "switch-sensor",
		.owner	= THIS_MODULE,
	},
};

static int __init sensor_switch_init(void)
{
	printk("sensor_switch module init\n");
	return platform_driver_register(&sensor_switch_driver);
}

static void __exit sensor_switch_exit(void)
{
	platform_driver_unregister(&sensor_switch_driver);
}

module_init(sensor_switch_init);
//late_initcall(sensor_switch_init);
module_exit(sensor_switch_exit);

MODULE_AUTHOR("Bob.yang <Bob.yang@infotmic.com.cn>");
/* modified by warits on apr.28 to fit new gpio structure */
MODULE_DESCRIPTION("SENSOR Switch driver");
MODULE_LICENSE("GPL");
