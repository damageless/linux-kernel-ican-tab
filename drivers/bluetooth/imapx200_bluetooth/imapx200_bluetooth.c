/***************************************************************************** 
 * imapx200_bluetooth.c
 * 
 * Copyright (c) 2009~2014 ShangHai Infotm Ltd all rights reserved. 
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * Description:
 * 	Main source file of IMAPX200 bluetooth driver.
 *
 * Author:
 *	Sololz <sololz.luo@gmail.com>.
 *      
 * Revision History: 
 * ­­­­­­­­­­­­­­­­­ 
 * 1.1  2011/01/06 Sololz
 * 	Create this file.
 ******************************************************************************/

#include "imapx200_bluetooth.h"

static int imapx200_bluetooth_probe(struct platform_device *pdev)
{
	/* TODO */
	return 0;
}

static int imapx200_bluetooth_remove(struct platform_device *pdev)
{
	/* TODO */
	return 0;
}

static int imapx200_bluetooth_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* TODO */
	return 0;
}

static int imapx200_bluetooth_resume(struct platform_device *pdev)
{
	/* TODO */
	return 0;
}

/* Platform driver. */
static struct platform_driver imapx200_bluetooth_platdev = {
	.probe = imapx200_bluetooth_probe,
	.remove = imapx200_bluetooth_remove,
	.suspend = imapx200_bluetooth_suspend,
	.resume = imapx200_bluetooth_resume,
	.driver = {
		.owner = THIS_MODULE,
		.name = "imapx200_bluetooth",
	},
};

#if 0
static int rda5868_bt_set_block(void *data, bool blocked)
{
	/* TODO */
	return 0;
}

/* RF driver. */
static struct rfkill_ops imapx200_bluetooth_rfkilldev = {
	.set_block = imapx200_bluetooth_set_block,
};
#endif

static int __init imapx200_bluetooth_init(void)
{
	return platform_driver_register(&imapx200_bluetooth_platdev);
}

static void __exit imapx200_bluetooth_exit(void)
{
	platform_driver_unregister(&imapx200_bluetooth_platdev);
}

/* module_init(imapx200_bluetooth_init); */
late_initcall(imapx200_bluetooth_init);
module_exit(imapx200_bluetooth_exit);

MODULE_AUTHOR("Sololz of InfoTM, <sololz.luo@gmail.com>");
MODULE_DESCRIPTION("RDA 5872p Bluetooth hardware control driver.");
MODULE_LICENSE("GPL");
