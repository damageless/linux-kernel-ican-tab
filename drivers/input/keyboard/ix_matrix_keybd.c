/*
 * linux/drivers/input/keyboard/pxa27x_keypad.c
 *
 * Driver for the pxa27x matrix keyboard controller.
 *
 * Created:	Feb 22, 2007
 * Author:	Rodolfo Giometti <giometti@linux.it>
 *
 * Based on a previous implementations by Kevin O'Connor
 * <kevin_at_koconnor.net> and Alex Osborne <bobofdoom@gmail.com> and
 * on some suggestions by Nicolas Pitre <nico@cam.org>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/hardware.h>
#include <linux/gpio.h>
#include "ix_matrix_keybd.h"


#define keypad_readl(off)	__raw_readl(keypad->mmio_base + (off))
#define keypad_writel(off, v)	__raw_writel((v), keypad->mmio_base + (off))
static int caps_locking = 0;
static spinlock_t *caps_spinlock = NULL;
#if defined(CONFIG_INPUT_KEYBOARD_SPANISH)
int span_key_locking = 0;
EXPORT_SYMBOL(span_key_locking);
static int span_key_mark = 0;
static spinlock_t *span_key_spinlock = NULL;
int span2_key_locking = 0;
EXPORT_SYMBOL(span2_key_locking);

static int span2_key_mark = 0;
static spinlock_t *span2_key_spinlock = NULL;

#endif
bool init = false;
static int shift_pressing = 0;
static int leftshift_pressing = 0;
static int rightshift_pressing = 0;
static int leftalt_pressing = 0;
static int rightalt_pressing = 0;

static int shift_key_locking = 0;
static spinlock_t *shift_key_spinlock = NULL;
static int alt_key_locking = 0;
static spinlock_t *alt_key_spinlock = NULL;
static int left_alt_locking = 0;
static int right_alt_locking = 0;
extern int imap_timer_setup(int channel,unsigned long g_tcnt,unsigned long gtcmp);
//extern void gpio_switch_keybd_work(void);
extern int current_intensity;
volatile int flag_NUML = 0;
volatile int flag_fn = 0;
volatile int flag_CAPS=0;
volatile int flag_shift=0;
unsigned int keybd_cs;
unsigned int led_capslock;
unsigned int led_numlock;

struct key_flag {
	int keycode;
	int scancode;
	int state;
};

static struct key_flag key_flags[] = {
	{ 
		.keycode = KEY_F1,
	        .scancode = KEY_VOLUMEUP,
	},
	{ 
		.keycode = KEY_F2,
	  	.scancode = KEY_VOLUMEDOWN,
	},
/*
	{ 
		.keycode = KEY_INSERT,
	  	.scancode = KEY_SYSRQ,
	},
*/
	{ 
		.keycode = KEY_LEFT,
	  	.scancode = KEY_HOME,
	},
	{ 
		.keycode = KEY_RIGHT,
	  	.scancode = KEY_END,
	},
	{ 
		.keycode = KEY_7,
	  	.scancode = KEY_7,
	},
	{ 
		.keycode = KEY_8,
	  	.scancode = KEY_8,
	},
	{ 	
		.keycode = KEY_9,
	  	.scancode = KEY_9,
	},
	{ 
		.keycode = KEY_0,
	  	.scancode = KEY_KPSLASH,
	},
	{ 
		.keycode = KEY_U,
	  	.scancode = KEY_4,
	},
	{ 
		.keycode = KEY_I,
	  	.scancode = KEY_5,
	},
	{ 
		.keycode = KEY_O,
	  	.scancode = KEY_6,
	},
	{ 
		.keycode = KEY_P,
	  	.scancode = KEY_KPASTERISK,
	},
	{ 
		.keycode = KEY_J,
	  	.scancode = KEY_1,
	},
	{ 
		.keycode = KEY_K,
	  	.scancode = KEY_2,
	},
	{ 	.keycode = KEY_L,
	 	.scancode = KEY_3,
	},
	{
		.keycode = KEY_M,
		.scancode = KEY_0,
	},
#if defined(CONFIG_INPUT_KEYBOARD_SPANISH)	
	{ 
		.keycode = KEY_F16,
	  	.scancode = KEY_MINUS,
	},
#elif defined(CONFIG_INPUT_KEYBOARD_AMERICA)
	{       
		 .keycode = KEY_SEMICOLON, 
		 .scancode = KEY_MINUS,
	},      
#endif
	{ 	
		.keycode = KEY_DOT,
	  	.scancode = KEY_DOT,
	},
#if defined(CONFIG_INPUT_KEYBOARD_SPANISH)
	{ 	
		.keycode = KEY_MINUS,
	  	.scancode = KEY_KPPLUS,
	},
#elif defined(CONFIG_INPUT_KEYBOARD_AMERICA)
	{       
		.keycode = KEY_SLASH,
	        .scancode = KEY_KPPLUS,
	},
#endif
};


int keyinput_get_caps_locking_status(void)
{
	int status = 0;

	spin_lock(caps_spinlock);
	status = caps_locking;
	printk("keyinput_get_caps_locking_status is %d\n",caps_locking);
	spin_unlock(caps_spinlock);

	return status;
}
EXPORT_SYMBOL(keyinput_get_caps_locking_status);
#if defined(CONFIG_INPUT_KEYBOARD_SPANISH)
int keyinput_get_span_key_locking_status(void)
{
	int status = 0;

	spin_lock(span_key_spinlock);
	status = span_key_locking;
	spin_unlock(span_key_spinlock);

	return status;
}
EXPORT_SYMBOL(keyinput_get_span_key_locking_status);
void keyinput_clear_span_key_locking_status(void)
{
	spin_lock(span_key_spinlock);
	span_key_locking = 0;
	spin_unlock(span_key_spinlock);
}
EXPORT_SYMBOL(keyinput_clear_span_key_locking_status);

int keyinput_get_span2_key_locking_status(void)
{
        int status = 0;

        spin_lock(span2_key_spinlock);
        status = span2_key_locking;
        spin_unlock(span2_key_spinlock);

        return status;
}
EXPORT_SYMBOL(keyinput_get_span2_key_locking_status);
void keyinput_clear_span2_key_locking_status(void)
{
        spin_lock(span2_key_spinlock);
        span2_key_locking = 0;
        spin_unlock(span2_key_spinlock);
}
EXPORT_SYMBOL(keyinput_clear_span2_key_locking_status);

#endif
int keyinput_get_key_shift_locking_status(void)
{
	int status = 0;
	spin_lock(shift_key_spinlock);
	status = shift_key_locking;
	spin_unlock(shift_key_spinlock);

	return status;
}
EXPORT_SYMBOL(keyinput_get_key_shift_locking_status);
int keyinput_get_key_alt_locking_status(void)
{
	int status = 0;
	spin_lock(alt_key_spinlock);
	status = alt_key_locking;
	spin_unlock(alt_key_spinlock);
		 
        return status;
}
EXPORT_SYMBOL(keyinput_get_key_alt_locking_status);
struct imapx200_keybd{
	struct imapx200_keybd_platform_data *pdata;

	struct input_dev *input_dev;
	struct clk *clk;

	void __iomem *mmio_base;

	int irq;

	unsigned int matrix_keycodes[MAX_MATRIX_KEY_NUM];
	unsigned int matrix_key_state[MAX_MATRIX_KEY_ROWS];

	int suspend;
};


int keyinput_enable_matrix_keyboard(void)
{
	imapx_gpio_setpin(keybd_cs, 0, IG_NORMAL);
	return;
}
EXPORT_SYMBOL(keyinput_enable_matrix_keyboard);

int keyinput_disable_matrix_keyboard(void)
{
	imapx_gpio_setpin(keybd_cs, 1, IG_NORMAL);
        return;
}
EXPORT_SYMBOL(keyinput_disable_matrix_keyboard);

static void imapx200_keybd_build_keycode(struct imapx200_keybd *keypad)
{
	struct imapx200_keybd_platform_data *pdata = keypad->pdata;
	struct input_dev *input_dev = keypad->input_dev;
	unsigned int *key;
	int i;

	key = &pdata->matrix_key_map[0];
	for (i = 0; i < pdata->matrix_key_map_size; i++, key++) {
		int row = ((*key) >> 24) & 0xff;
		int col = ((*key) >> 20) & 0xf;
		int code = (*key) & 0xfffff;

		keypad->matrix_keycodes[(row << 3) + col] = code;
//		printk("scancode...row:%x,col:%x....keycode:%x\r\n",row,col,code);
		set_bit(code, input_dev->keybit);
	}
}

static inline unsigned int lookup_matrix_keycode(
		struct imapx200_keybd *keypad, int row, int col)
{
	return keypad->matrix_keycodes[(row << 3) + col];
}

static void imapx200_keybd_scan_matrix(struct imapx200_keybd *keypad)
{
	struct imapx200_keybd_platform_data *pdata = keypad->pdata;
	int iRow, iCol;
	uint32_t kbRowData[MAX_MATRIX_KEY_ROWS];
	volatile int key;
	int intensity;
	int index = 0;
	memset(kbRowData, 0, sizeof(kbRowData));
	kbRowData[0] = keypad_readl(rKBROWD0) & 0xff;
	kbRowData[1] = (keypad_readl(rKBROWD0) & (0xff<<8))>>8;
        kbRowData[2] = (keypad_readl(rKBROWD0) & (0xff<<16))>>16;
        kbRowData[3] = (keypad_readl(rKBROWD0) & (0xff<<24))>>24;
        kbRowData[4] = keypad_readl(rKBROWD1) & 0xff;
        kbRowData[5] = (keypad_readl(rKBROWD1) & (0xff<<8))>>8;
        kbRowData[6] = (keypad_readl(rKBROWD1) & (0xff<<16))>>16;
        kbRowData[7] = (keypad_readl(rKBROWD1) & (0xff<<24))>>24;
        kbRowData[8] = keypad_readl(rKBROWD2) & 0xff;
        kbRowData[9] = (keypad_readl(rKBROWD2) & (0xff<<8))>>8;
        kbRowData[10] = (keypad_readl(rKBROWD2) & (0xff<<16))>>16;
        kbRowData[11] = (keypad_readl(rKBROWD2) & (0xff<<24))>>24;
        kbRowData[12] = keypad_readl(rKBROWD3) & 0xff;
        kbRowData[13] = (keypad_readl(rKBROWD3) & (0xff<<8))>>8;
        kbRowData[14] = (keypad_readl(rKBROWD3) & (0xff<<16))>>16;
        kbRowData[15] = (keypad_readl(rKBROWD3) & (0xff<<24))>>24;
        kbRowData[16] = keypad_readl(rKBROWD4) & 0xff;
        kbRowData[17] = (keypad_readl(rKBROWD4) & (0xff<<8))>>8;

	for (iRow = 0; iRow < pdata->matrix_key_rows; iRow++) 
	{
		uint32_t bits_changed;

		bits_changed = keypad->matrix_key_state[iRow] ^ kbRowData[iRow];
		if (bits_changed == 0)
			continue;
		for (iCol = 0; iCol < pdata->matrix_key_cols; iCol++)
		{
			if ((bits_changed & (1 << iCol)) == 0)
				continue;
			key =lookup_matrix_keycode(keypad, iRow, iCol);
			//printk("....................iRow is %d.....................\n",iRow);
			//printk("....................iCol is %d.....................\n",iCol);
			//printk("....................key is %d......................\n",key);
			if (key == KEY_FN)
			{
				 flag_fn++;
				 if (flag_fn == 3) {
					for (index = 0;index < sizeof(key_flags)/sizeof(key_flags[0]);index ++)
						if (key_flags[index].state) {
							key_flags[index].state = 0;
							input_report_key(keypad->input_dev, key_flags[index].scancode, 0);
						}
				 }				        
				 if( flag_fn == 3)
				         flag_fn = 1;

				 //printk("...................flag_fn is ok...............\n");
			}
			//printk("....................KEY_FN is %d...................\n",flag_fn);
			if (!(kbRowData[iRow] & (1 << iCol)) && (key == KEY_CAPSLOCK)) {
				spin_lock(caps_spinlock);
			        if (caps_locking) {
				        caps_locking = 0;
			                //printk(KERN_ALERT "Capslock is disabled.\n"); 
			        } else {
			                caps_locking = 1;
				        //printk(KERN_ALERT "Capslock is enabled.\n");
				}
				spin_unlock(caps_spinlock);
		        }
			if(key == KEY_CAPSLOCK)
			{
				flag_CAPS++;
				if(flag_CAPS==2)
				{
					flag_CAPS=0;
					imapx_gpio_setpin(led_capslock,
					!imapx_gpio_getpin(led_capslock, IG_NORMAL), IG_NORMAL);
				}
			}
			if (key == KEY_LEFTSHIFT) {
				if (!(kbRowData[iRow] & (1 << iCol))) {
				        leftshift_pressing = 1; 
				} else {
				        leftshift_pressing = 0; 
				}    
			}    
		        if (key == KEY_RIGHTSHIFT) {
			        if (!(kbRowData[iRow] & (1 << iCol))) {
			                rightshift_pressing = 1; 
			        } else {
			                rightshift_pressing = 0; 
			        }    
			}    

			if (key == KEY_LEFTALT) {
			        if (!(kbRowData[iRow] & (1 << iCol))) {
			                leftalt_pressing = 1; 
			        } else {
			                leftalt_pressing = 0; 
			        }    
			}    
			if (key == KEY_RIGHTALT) {
			        if (!(kbRowData[iRow] & (1 << iCol))) {
			                rightalt_pressing = 1; 
			        } else {
			                rightalt_pressing = 0; 
			        }
			}
			if (key == KEY_LEFTSHIFT ||key == KEY_RIGHTSHIFT) {
				spin_lock(shift_key_spinlock);
				if (leftshift_pressing || rightshift_pressing) {
					shift_key_locking = 1;
				} else if (shift_key_locking) {
					shift_key_locking = 0;
				}
				spin_unlock(shift_key_spinlock);
			}

			if (key == KEY_LEFTALT ||key == KEY_RIGHTALT) {
				spin_lock(alt_key_spinlock);
				if (rightalt_pressing || leftalt_pressing) {
					alt_key_locking = 1;
				} else if (alt_key_locking) {
					alt_key_locking = 0;
				}
				spin_unlock(alt_key_spinlock);
			}
#if defined(CONFIG_INPUT_KEYBOARD_SPANISH)	
			/* KEY_APOSTROPHE is used to do spankey status record. */
			if (key == KEY_F17) {
			        if (!(kbRowData[iRow] & (1 << iCol))) {
			                spin_lock(span_key_spinlock);
					span2_key_locking = 0;
			        	if (span_key_locking) {
			                	span_key_locking = 0; 
			                	spin_unlock(span_key_spinlock);
						input_report_key(keypad->input_dev,span_key_mark,1);
						input_report_key(keypad->input_dev,span_key_mark,0);
					} else {
			                	if (leftalt_pressing || rightalt_pressing) {
			                		spin_unlock(span_key_spinlock);
							input_report_key(keypad->input_dev,KEY_F17,1);
							input_report_key(keypad->input_dev,KEY_F17,0);
							input_sync(keypad->input_dev);
							memcpy(keypad->matrix_key_state, kbRowData, sizeof(kbRowData));
					        	return;
				        	}

	                                	if (leftshift_pressing || rightshift_pressing) {
	                                        	span_key_mark = KEY_F19;
	                                        	span_key_locking = 2;
	                                	} else {
	                                        	span_key_mark = KEY_F17;
	                                        	span_key_locking = 1;
	                                	}
	                                	spin_unlock(span_key_spinlock);
	                        	}
	                	}
				if (init) {
					input_sync(keypad->input_dev);
					memcpy(keypad->matrix_key_state, kbRowData, sizeof(kbRowData));
		                	return;
				}
		        }
	
#if 0
			if (key == KEY_GRAVE) {
			        if (!(kbRowData[iRow] & (1 << iCol))) {
			                spin_lock(span2_key_spinlock);
					span_key_locking = 0;
			                if (span2_key_locking) {
			                        span2_key_locking = 0;
			                        spin_unlock(span2_key_spinlock);
			                        input_report_key(keypad->input_dev,span2_key_mark,1);
			                        input_report_key(keypad->input_dev,span2_key_mark,0);
			                } else {
			                        if (leftalt_pressing || rightalt_pressing) {
			                                spin_unlock(span2_key_spinlock);
			                                input_report_key(keypad->input_dev,KEY_GRAVE,1);
			                                input_report_key(keypad->input_dev,KEY_GRAVE,0);
			                                input_sync(keypad->input_dev);
			                                memcpy(keypad->matrix_key_state, kbRowData, sizeof(kbRowData));
			                                return;
			                        }

			                        if (leftshift_pressing || rightshift_pressing) {
			                                span2_key_mark = KEY_F20;
			                                span2_key_locking = 2;
			                        } else {
			                                span2_key_mark = KEY_GRAVE;
			                                span2_key_locking = 1;
			                        }
					        spin_unlock(span2_key_spinlock);
					}
			       }
			       if (init) {
			       		input_sync(keypad->input_dev);
			       		memcpy(keypad->matrix_key_state, kbRowData, sizeof(kbRowData));
			       		return;
			       }
			}
#endif
			//printk("span_key_locking is %d\n",span_key_locking);
			/* This part of code is without lock protection but nothing matters. */
			if (!(kbRowData[iRow] & (1 << iCol)) && (key != KEY_F17) ) {
				if (span_key_locking) {
					switch (key) {
						case KEY_A:
						case KEY_E:
						case KEY_I:
					        case KEY_O:
						case KEY_U:
						case KEY_LEFTSHIFT:
						case KEY_RIGHTSHIFT:
						case KEY_LEFTALT:
						case KEY_RIGHTALT:
						case KEY_CAPSLOCK:
						case KEY_ENTER:
						case KEY_BACKSPACE:
						case KEY_ESC:
						case KEY_LEFTCTRL:
						case KEY_RIGHTCTRL:
						case KEY_MENU:
						case KEY_HOME:
						case KEY_UP:
						case KEY_DOWN:
						case KEY_LEFT:
						case KEY_RIGHT:
						case KEY_F1:
						case KEY_F2:
						case KEY_F3:
						case KEY_F4:
						case KEY_F5:
						case KEY_F6:
						case KEY_F7:
						case KEY_F8:
						case KEY_F9:
						case KEY_F10:
						case KEY_F11:
						case KEY_F12:
						case KEY_VOLUMEDOWN:
						case KEY_VOLUMEUP:
						case KEY_NUMLOCK:
						case KEY_DELETE:
						case KEY_END:
							break;

						case KEY_SPACE:
							input_report_key(keypad->input_dev,span_key_mark,1);
							input_report_key(keypad->input_dev,span_key_mark,0);
							input_sync(keypad->input_dev);
							memcpy(keypad->matrix_key_state, kbRowData, sizeof(kbRowData));
							return;

						default:
							input_report_key(keypad->input_dev,span_key_mark,1);
							input_report_key(keypad->input_dev,span_key_mark,0);
							break;
					}
				} 
#if 0
				if (span2_key_locking) {
					switch (key) {
						case KEY_A:
						case KEY_E:
						case KEY_I:
						case KEY_O:
						case KEY_U:
						case KEY_LEFTSHIFT:
						case KEY_RIGHTSHIFT:
						case KEY_LEFTALT:
						case KEY_RIGHTALT:
						case KEY_CAPSLOCK:
						case KEY_ENTER:
						case KEY_BACKSPACE:
						case KEY_ESC:
						case KEY_LEFTCTRL:
						case KEY_RIGHTCTRL:
						case KEY_MENU:
						case KEY_HOME:
						case KEY_UP:
						case KEY_DOWN:
						case KEY_LEFT:
						case KEY_RIGHT:
						case KEY_F1:
						case KEY_F2:
						case KEY_F3:
						case KEY_F4:
						case KEY_F5:
						case KEY_F6:
						case KEY_F7:
						case KEY_F8:
						case KEY_F9:
						case KEY_F10:
						case KEY_F11:
						case KEY_F12:
						case KEY_VOLUMEDOWN:
						case KEY_VOLUMEUP:
						case KEY_NUMLOCK:
						case KEY_DELETE:
						case KEY_END:
							        break;

						case KEY_SPACE:
							input_report_key(keypad->input_dev,span2_key_mark,1);
							input_report_key(keypad->input_dev,span2_key_mark,0);
							input_sync(keypad->input_dev);
							memcpy(keypad->matrix_key_state, kbRowData, sizeof(kbRowData));
							return;

						default:
							input_report_key(keypad->input_dev,span2_key_mark,1);
							input_report_key(keypad->input_dev,span2_key_mark,0);
							break;
					}
				}
#endif
			}
 	
#endif
			//printk(".....................KEY_shift is %d...................\n",flag_shift);
			if(key == KEY_LEFTSHIFT||key == KEY_RIGHTSHIFT)
			{	
				flag_shift++;
				if(flag_shift==4)
					flag_shift=2;
			}
			//printk("....................KEY_CAPSLOCK is %d...................\n",flag_CAPS);
			//printk(".....................KEY_shift is %d...................\n",flag_shift);	
			if (flag_fn == 2)
			{

				for (index = 0; index < sizeof(key_flags) / sizeof(key_flags[0]); index++) {
					if (key_flags[index].keycode == key) {
						if (key_flags[index].state)
							key_flags[index].state = 0;
						else
							key_flags[index].state = 1;
					input_report_key(keypad->input_dev, key_flags[index].scancode,
						!(kbRowData[iRow] & (1 << iCol)));
					}

				}
			
			if (key == KEY_F11)
				if(!(kbRowData[iRow] & (1 << iCol)))
				{
					if(flag_NUML)
						flag_NUML = 0;
					else
						flag_NUML = 1;

					input_report_key(keypad->input_dev,KEY_NUMLOCK,
						!(kbRowData[iRow] & (1 << iCol)));
					imapx_gpio_setpin(led_numlock,
					!imapx_gpio_getpin(led_numlock, IG_NORMAL), IG_NORMAL);
				}
		        }
			if (flag_NUML == 1)
			{
				//printk("NumLock is set!\r\n");
				if (flag_fn!=2) { 	
					switch( key )
					{
					case KEY_7:
						input_report_key(keypad->input_dev,KEY_KP7,
							!(kbRowData[iRow] & (1 << iCol)));
						//printk(".....................!(kbRowData[iRow] & (1 << iCol)) is %d.....................\n",!(kbRowData[iRow] & (1 << iCol)));
						break;
					case KEY_8:
						input_report_key(keypad->input_dev,KEY_KP8,
							!(kbRowData[iRow] & (1 << iCol)));
						break;
					case KEY_9:
						input_report_key(keypad->input_dev,KEY_KP9,
							!(kbRowData[iRow] & (1 << iCol)));
						break;
					case KEY_0:
						input_report_key(keypad->input_dev,KEY_KPSLASH,
							!(kbRowData[iRow] & (1 << iCol)));
						break;
					case KEY_U:
						input_report_key(keypad->input_dev,KEY_KP4,
							!(kbRowData[iRow] & (1 << iCol)));
						break;
					case KEY_I:
						input_report_key(keypad->input_dev,KEY_KP5,
							!(kbRowData[iRow] & (1 << iCol)));
						break;
					case KEY_O:
						input_report_key(keypad->input_dev,KEY_KP6,
							!(kbRowData[iRow] & (1 << iCol)));
						break;
					case KEY_P:
						input_report_key(keypad->input_dev,KEY_KPASTERISK,
							!(kbRowData[iRow] & (1 << iCol)));
						break;
					case KEY_J:
						input_report_key(keypad->input_dev,KEY_KP1,
							!(kbRowData[iRow] & (1 << iCol)));
						break;
					case KEY_K:
						input_report_key(keypad->input_dev,KEY_KP2,
							!(kbRowData[iRow] & (1 << iCol)));
						break;
					case KEY_L:
						input_report_key(keypad->input_dev,KEY_KP3,
							!(kbRowData[iRow] & (1 << iCol)));
						break;
#if defined(CONFIG_INPUT_KEYBOARD_SPANISH)	
					case KEY_F16:
						input_report_key(keypad->input_dev,KEY_KPMINUS,
							!(kbRowData[iRow] & (1 << iCol)));
						break;
#elif defined(CONFIG_INPUT_KEYBOARD_AMERICA)
					case KEY_SEMICOLON:
						input_report_key(keypad->input_dev,KEY_KPMINUS,
							!(kbRowData[iRow] & (1 << iCol)));
						break;
#endif
					case KEY_M:
						input_report_key(keypad->input_dev,KEY_KP0,
							!(kbRowData[iRow] & (1 << iCol)));
						break;
					case KEY_DOT:
						input_report_key(keypad->input_dev,KEY_KPDOT,
							!(kbRowData[iRow] & (1 << iCol)));
						break;
#if defined(CONFIG_INPUT_KEYBOARD_SPANISH)
					case KEY_MINUS:
						input_report_key(keypad->input_dev,KEY_F18,
							!(kbRowData[iRow] & (1 << iCol)));
						break;
#elif defined(CONFIG_INPUT_KEYBOARD_AMERICA)
					case KEY_SLASH:
						input_report_key(keypad->input_dev,KEY_F18,
							!(kbRowData[iRow] & (1 << iCol)));
						break;
#endif
					default:
						if(flag_fn!=2)
						input_report_key(keypad->input_dev,
							lookup_matrix_keycode(keypad, iRow, iCol),
							!(kbRowData[iRow] & (1 << iCol)));
						//printk("............3...................\n");
						break;
					}
				}
			}
			if (key == KEY_LEFTALT){
				if (!(kbRowData[iRow] & (1 << iCol)) == 1 ) {
					left_alt_locking++;
				} else {
				        left_alt_locking=0;     
				}
		        }
		        if (key == KEY_RIGHTALT){
		                if (!(kbRowData[iRow] & (1 << iCol)) == 1 ) 
		                        right_alt_locking++;
		                else
		                        right_alt_locking=0;
		        }
		
	        	if((left_alt_locking!=0||right_alt_locking!=0)&&(key == KEY_LEFT||key == KEY_RIGHT))
	        	//printk("....................alt_locking is ok\n");
	               		;
	        	else if (key == KEY_LEFTCTRL && !(kbRowData[iRow] & (1 << iCol)) == 1)
	               		;
	        	else if ((left_alt_locking!=0||right_alt_locking!=0)&&key == KEY_F1&&!(kbRowData[iRow] & (1 << iCol))) {
				input_report_key(keypad->input_dev,KEY_LEFTALT,0);
				input_report_key(keypad->input_dev,
						lookup_matrix_keycode(keypad, iRow, iCol),
						!(kbRowData[iRow] & (1 << iCol)));
	        	} else
				if(flag_NUML!=1&&flag_fn!=2) 
				{	
					input_report_key(keypad->input_dev,
							lookup_matrix_keycode(keypad, iRow, iCol),
							!(kbRowData[iRow] & (1 << iCol)));
				}
		}
	}
	init = true;
	input_sync(keypad->input_dev);
	memcpy(keypad->matrix_key_state, kbRowData, sizeof(kbRowData));
}


static irqreturn_t imapx200_keybd_irq_handler(int irq, void *dev_id)
{
	struct imapx200_keybd *keypad = dev_id;
		
	//clear keybd interrupt
	keypad_writel(rKBINT , 0x1ffff);

	while(1)
	{
		if(keypad_readl(rKBINT) == 0)
		{
			break;
		}
		else
		{
			 keypad_writel(rKBINT , 0x1ffff);
		}
	}

	imapx200_keybd_scan_matrix(keypad);

//	//clear keybd interrupt
//	keypad_writel(rKBINT , KBDCNT_DRDYINT);	
	return IRQ_HANDLED;
}

static int imapx200_keybd_open(struct input_dev *dev)
{
	struct imapx200_keybd *keypad = input_get_drvdata(dev);
	
	/* Enable unit clock */
	clk_enable(keypad->clk);

	/* enable matrix keys with automatic scan */
	keypad_writel(rKBCKD , 1024);
	keypad_writel(rKBDCNT , 100);
	keypad_writel(rKBCOLD , 0);

	keypad_writel(rKBRPTC , 1024*6);	
	keypad_writel(rKBCON , (KBCON_RPTEN |KBCON_FCEN| KBCON_DFEN | KBCON_DRDYINTEN));
	keypad_writel(rKBCOEN , (KBCOEN_COLNUM | KBCOEN_COLOEN));

        //clear keybd interrupt
        keypad_writel(rKBINT , 0x1ffff);
         while(1)
         {
                 if(keypad_readl(rKBINT) == 0)
                 {
                         break;
                 }
                 else
                 {
                          keypad_writel(rKBINT , 0x1ffff);
                 }
         }
	
	return 0;
}

static void imapx200_keybd_close(struct input_dev *dev)
{
	struct imapx200_keybd *keypad = input_get_drvdata(dev);
	
	/* Disable clock unit */
	clk_disable(keypad->clk);
}

//#ifdef CONFIG_PM
#if 0
static int imapx200_keybd_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct imapx200_keybd *keypad = platform_get_drvdata(pdev);

	clk_disable(keypad->clk);

	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(keypad->irq);

	return 0;
}

static int imapx200_keybd_resume(struct platform_device *pdev)
{
	struct imapx200_keybd *keypad = platform_get_drvdata(pdev);
	struct input_dev *input_dev = keypad->input_dev;

	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(keypad->irq);

	mutex_lock(&input_dev->mutex);

	if (input_dev->users) {
		/* Enable unit clock */
		clk_enable(keypad->clk);
		pxa27x_keypad_config(keypad);
	}

	mutex_unlock(&input_dev->mutex);

	return 0;
}
#else
#define imapx200_keybd_suspend	NULL
#define imapx200_keybd_resume	NULL
#endif

#define res_size(res)	((res)->end - (res)->start + 1)

static int __devinit imapx200_keybd_probe(struct platform_device *pdev)
{
	struct imapx200_keybd *keypad;
	struct input_dev *input_dev;
	struct resource *res;
	int irq, error, i;
	int gphcon, gpicon, gpjcon,gpocon,gpmcon,gplcon;

	//keyboard enable/disable
	keybd_cs = __imapx_name_to_gpio(CONFIG_KEYBOARD_MATRIX_CS);
        if(keybd_cs == IMAPX_GPIO_ERROR) {
                printk(KERN_ERR "failed to get keybd_cs pin.\n");
                return -1;
        }   
        imapx_gpio_setcfg(keybd_cs, IG_OUTPUT, IG_NORMAL);
	
	//led_capslock control
	led_capslock = __imapx_name_to_gpio(CONFIG_KEYBOARD_LED_CAPS);
        if(led_capslock == IMAPX_GPIO_ERROR) {
                printk(KERN_ERR "failed to get led_capslock pin.\n");
                return -1;
        }
        imapx_gpio_setcfg(led_capslock, IG_OUTPUT, IG_NORMAL);
	imapx_gpio_setpin(led_capslock, 0, IG_NORMAL);

	
	//led_numlock control
	led_numlock = __imapx_name_to_gpio(CONFIG_KEYBOARD_LED_NUML);	
        if(led_numlock == IMAPX_GPIO_ERROR) {
                printk(KERN_ERR "failed to get led_numlock pin.\n");
                return -1;
        }
        imapx_gpio_setcfg(led_numlock, IG_OUTPUT, IG_NORMAL);
        imapx_gpio_setpin(led_numlock, 0, IG_NORMAL);
	
	//config the GPIO port for keyboard 
	imapx_gpio_setcfg(IMAPX_GPH_RANGE(0,3), IG_CTRL1, IG_NORMAL);
	imapx_gpio_setcfg(IMAPX_GPI_RANGE(0,13), IG_CTRL1, IG_NORMAL);
	imapx_gpio_setcfg(IMAPX_GPJ_RANGE(0,8), IG_CTRL1, IG_NORMAL);

	pdev->dev.platform_data = &imapx200_keybd_info;

	keypad = kzalloc(sizeof(struct imapx200_keybd), GFP_KERNEL);
	if (keypad == NULL) {
		dev_err(&pdev->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	keypad->pdata = pdev->dev.platform_data;
	if (keypad->pdata == NULL) {
		dev_err(&pdev->dev, "no platform data defined\n");
		error = -EINVAL;
		goto failed_free;
	}

	//memset(keypad->matrix_key_state,0,sizeof(keypad->matrix_key_state));

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to get keypad irq\n");
		error = -ENXIO;
		goto failed_free;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get I/O memory\n");
		error = -ENXIO;
		goto failed_free;
	}

	res = request_mem_region(res->start, res_size(res), pdev->name);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to request I/O memory\n");
		error = -EBUSY;
		goto failed_free;
	}

	keypad->mmio_base = ioremap(res->start, res_size(res));
	if (keypad->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to remap I/O memory\n");
		error = -ENXIO;
		goto failed_free_mem;
	}

        keypad->clk = clk_get(&pdev->dev, "kb");
	if (IS_ERR(keypad->clk)) {
		dev_err(&pdev->dev, "failed to get keypad clock\n");
		error = PTR_ERR(keypad->clk);
		goto failed_free_io;
	}
	if (caps_spinlock == NULL) {
		caps_spinlock = (spinlock_t *)kmalloc(sizeof(spinlock_t), GFP_KERNEL);
		if (caps_spinlock == NULL) {
			printk(KERN_ERR "kmalloc() for capslock key spinlock error.\n");
			return -1;
		}
		spin_lock_init(caps_spinlock);
	}
#if defined(CONFIG_INPUT_KEYBOARD_SPANISH)
	if (span_key_spinlock == NULL) {
		span_key_spinlock = (spinlock_t *)kmalloc(sizeof(spinlock_t), GFP_KERNEL);
		if (span_key_spinlock == NULL) {
			printk(KERN_ERR "kmalloc() for span key spinlock error.\n");
			return -1;
		}
		spin_lock_init(span_key_spinlock);
	}

	if (span2_key_spinlock == NULL) {
	        span2_key_spinlock = (spinlock_t *)kmalloc(sizeof(spinlock_t), GFP_KERNEL);
	        if (span2_key_spinlock == NULL) {
	                printk(KERN_ERR "kmalloc() for span2 key spinlock error.\n");
	                return -1;
	        }
	        spin_lock_init(span2_key_spinlock);
	}

#endif
	if (shift_key_spinlock ==NULL) {
		shift_key_spinlock = (spinlock_t *)kmalloc(sizeof(spinlock_t), GFP_KERNEL);
		if (shift_key_spinlock == NULL) {
			printk(KERN_ERR "kmalloc() for shift key spinlock error.\n");
			return -1;
		}
		spin_lock_init(shift_key_spinlock);
	}
	if (alt_key_spinlock ==NULL) {
		alt_key_spinlock = (spinlock_t *)kmalloc(sizeof(spinlock_t), GFP_KERNEL);
		if (alt_key_spinlock == NULL) {
			printk(KERN_ERR "kmalloc() for alt key spinlock error.\n");
			return -1;
		}
			spin_lock_init(alt_key_spinlock);
	}

	/* Create and register the input driver. */
	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&pdev->dev, "failed to allocate input device\n");
		error = -ENOMEM;
		goto failed_put_clk;
	}

	input_dev->name = pdev->name;
	input_dev->id.bustype = BUS_HOST;
	input_dev->open = imapx200_keybd_open;
	input_dev->close = imapx200_keybd_close;
	input_dev->dev.parent = &pdev->dev;

	keypad->input_dev = input_dev;
	input_set_drvdata(input_dev, keypad);

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP);

	imapx200_keybd_build_keycode(keypad);

	platform_set_drvdata(pdev, keypad);

	error = request_irq(irq, imapx200_keybd_irq_handler, IRQF_DISABLED,
			    pdev->name, keypad);
	if (error) {
		dev_err(&pdev->dev, "failed to request IRQ\n");
		goto failed_free_dev;
	}

	keypad->irq = irq;

	/* Register the input device */
	error = input_register_device(input_dev);
	if (error) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto failed_free_irq;
	}

	device_init_wakeup(&pdev->dev, 1);

	return 0;

failed_free_irq:
	free_irq(irq, pdev);
	platform_set_drvdata(pdev, NULL);
failed_free_dev:
	input_free_device(input_dev);
failed_put_clk:
	clk_put(keypad->clk);
failed_free_io:
	iounmap(keypad->mmio_base);
failed_free_mem:
	release_mem_region(res->start, res_size(res));
failed_free:
	kfree(keypad);
	return error;
}

static int __devexit imapx200_keybd_remove(struct platform_device *pdev)
{
	struct imapx200_keybd *keypad = platform_get_drvdata(pdev);
	struct resource *res;

	free_irq(keypad->irq, pdev);

	clk_disable(keypad->clk);
	clk_put(keypad->clk);

	input_unregister_device(keypad->input_dev);
	input_free_device(keypad->input_dev);

	iounmap(keypad->mmio_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, res_size(res));

	platform_set_drvdata(pdev, NULL);
	kfree(keypad);
	return 0;
}

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:imapx200_keypad");

static struct platform_driver imapx200_keybd_driver = {
	.probe		= imapx200_keybd_probe,
	.remove		= __devexit_p(imapx200_keybd_remove),
	.suspend	= imapx200_keybd_suspend,
	.resume		= imapx200_keybd_resume,
	.driver		= {
		.name	= "imapx200_keybd",
		.owner	= THIS_MODULE,
	},
};

static int __init imapx200_keybd_init(void)
{
	return platform_driver_register(&imapx200_keybd_driver);
}

static void __exit imapx200_keybd_exit(void)
{
	platform_driver_unregister(&imapx200_keybd_driver);
}

module_init(imapx200_keybd_init);
module_exit(imapx200_keybd_exit);

MODULE_DESCRIPTION("imapx200 keybd Controller Driver");
MODULE_LICENSE("GPL");


