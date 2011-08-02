/***********************************************************************************
 * soops.c
 *
 * Copyright (c) 2009~2014 ShangHai Infotm Ltd all rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Description:
 * 	A complicated char device driver created by Sololz first designed 
 * 	for Android Capslock key of USB keyboard support.
 *
 * Author:
 * 	Sololz <sololz.luo@gmail.com>.
 *
 * Revision History:
 * 1.0	2010/12/28 Sololz
 * 	Create this file, support Capslock key.
 * 1.1	2011/01/12 Sololz
 * 	Add special Spanish key support.
 **********************************************************************************/

#include "soops.h"

/* EXTERNAL FUNCTION DELCARATION. */
extern int usbinput_get_caps_locking_status(void);
#if defined(CONFIG_KEYBOARD_MATRIX)
extern int keyinput_get_caps_locking_status(void);
extern int keyinput_enable_matrix_keyboard(void);
#endif

#if defined(CONFIG_KEYBOARD_MATRIX)
extern int keyinput_get_key_shift_locking_status(void);
extern int keyinput_get_key_alt_locking_status(void);
#endif
extern int usbinput_get_key_shift_locking_status(void);
extern int usbinput_get_key_alt_locking_status(void);
static int hidinput_get_key_shift_locking_status(void)
{
       int tmp1,tmp2=0,tmp;
       tmp1 = usbinput_get_key_shift_locking_status();
#if defined(CONFIG_KEYBOARD_MATRIX)
       tmp2 = keyinput_get_key_shift_locking_status();
#endif 
       tmp = tmp1 || tmp2;

       return tmp;
}
static int hidinput_get_key_alt_locking_status(void)
{
	int tmp1,tmp2=0,tmp;
	tmp1 = usbinput_get_key_alt_locking_status();
#if defined(CONFIG_KEYBOARD_MATRIX)
	tmp2 = keyinput_get_key_alt_locking_status();
#endif
	tmp = tmp1 || tmp2;

	return tmp;
}
static int hidinput_get_caps_locking_status(void)
{       
       int tmp1,tmp2=0,tmp;
       tmp1 = usbinput_get_caps_locking_status();
#if defined(CONFIG_KEYBOARD_MATRIX)
       tmp2 = keyinput_get_caps_locking_status();
#endif
       tmp = (tmp1^tmp2);
       //printk("...................get_caps is %d\n",tmp);
       return tmp;
}


#if defined(CONFIG_INPUT_KEYBOARD_SPANISH)
extern int span_key_locking;
static int hidinput_get_span_key_locking_status(void)
{
	return span_key_locking;
}
extern void usbinput_clear_span_key_locking_status(void);
#if defined(CONFIG_KEYBOARD_MATRIX)
extern void keyinput_clear_span_key_locking_status(void);
#endif
#endif
/* ############################################################################## */

/* 
 * FIXME: 
 * This system call function should not be designed to be too much feature relates.
 */
static int soops_open(struct inode *inode, struct file *file)
{
	return 0;
}

/* 
 * FIXME: 
 * This system call function should not be designed to be too much feature relates.
 */
static int soops_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int soops_ioctl(struct inode *inode, struct file *file, \
		unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	/* Check IO control command access permission validation. */
	if (_IOC_TYPE(cmd) != SOOPS_IOCMD_MAGIC) {
		soops_error("Unknow IO control command magic.\n");
		return -EINVAL;
	} else if (_IOC_NR(cmd) > SOOPS_IOCMD_MAX_NUM) {
		soops_error("Overflow IO control index.\n");
		return -EINVAL;
	}

	if (_IOC_DIR(cmd) & _IOC_READ) {
		if (!access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd))) {
			soops_error("IO control request read but buffer unwritable.\n");
			return -EINVAL;
		}
	} else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (!access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd))) {
			soops_error("IO control request write but buffer unreadable.\n");
			return -EINVAL;
		}
	}

	/* Process all IO control requests. */
	switch (cmd) {
		case SOOPS_IOCMD_GET_CAPS_STATUS:
			__put_user(hidinput_get_caps_locking_status(), (int *)arg);
			break;
#if defined(CONFIG_INPUT_KEYBOARD_SPANISH)
		case SOOPS_IOCMD_GET_SPANKEY_STATUS:
			__put_user(hidinput_get_span_key_locking_status(), (int *)arg);
			break;

		case SOOPS_IOCMD_CLEAR_SPANKEY_STATUS:
			usbinput_clear_span_key_locking_status();
#if defined(CONFIG_KEYBOARD_MATRIX)	
			keyinput_clear_span_key_locking_status();
#endif
			break;
#endif

		case SOOPS_IOCMD_TEST:
			break;
#if defined(CONFIG_KEYBOARD_MATRIX)
		case SOOPS_IOCMD_ENABLE_MATRIX_KEYBOARD:
			keyinput_enable_matrix_keyboard();
#endif
		case SOOPS_IOCMD_GET_SHIFTKEY_STATUS:
			__put_user(hidinput_get_key_shift_locking_status(), (int *)arg);
			break;
		case SOOPS_IOCMD_GET_ALTKEY_STATUS:
			__put_user(hidinput_get_key_alt_locking_status(), (int *)arg);
			break;
		default:
			soops_error("Unknown IO control command.\n");
			ret = -EINVAL;
			break;
	}

	return ret;
}

static struct file_operations soops_ops = 
{
	owner: THIS_MODULE,
	open: soops_open,
	release: soops_release,
	ioctl: soops_ioctl,
};

static struct miscdevice soops_miscdev = 
{
	minor: MISC_DYNAMIC_MINOR,
	name: SOOPS_DEV_NAME,
	fops: &soops_ops
};

static int __init soops_init(void)
{
	return misc_register(&soops_miscdev);
}

static void __exit soops_exit(void)
{
	misc_deregister(&soops_miscdev);
}

module_init(soops_init);
module_exit(soops_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sololz of InfoTM");

