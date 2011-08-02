/***************************************************************************** 
 * ** drivers/video/infotm_HDMI/imap_HDMI.c
 * ** 
 * ** Copyright (c) 2009~2014 ShangHai Infotm Ltd all rights reserved. 
 * ** 
 * ** Use of Infotm's code is governed by terms and conditions 
 * ** stated in the accompanying licensing statement. 
 * ** 
 * ** Description: Implementation file of Infotm HDMI.
 * **
 * ** Author:
 * **     Alex Zhang <alex.zhang@infotmic.com.cn>
 * **      
 * ** Revision History: 
 * ** ----------------- 
 * ** 1.0  06/11/2010 Alex Zhang 
 * ** 1.1  06/18/2010 Alex Zhang 
 * ** 2.0  06/21/2010 Alex Zhang 
 * ** 2.1  06/23/2010 Alex Zhang 
 * ** 2.2  06/24/2010 Alex Zhang 
 * *****************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/ioctl.h>
#include <linux/poll.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <plat/imapx.h>
#include <asm/mach/irq.h>
/* import imapfb configuration for compatibility */
#include "../../infotm/imapfb.h"
#include "imap_HDMI.h"
#include "SIITPI.h"
#include "SIITPI_Access.h"
#include "Externals.h"
#include "SIITPI_Regs.h"
#include "SIIConstants.h"
#include "SIIMacros.h"
#include "SIIedid.h"
#include "SIIdefs.h"
#include "SIIAV_Config.h"

#ifdef CONFIG_FAKE_PM
#include <plat/fake_pm.h>
#endif

#define HDMI_MAJOR               84
#define HDMI_POLL_MAJOR               85
#define HDMI_MINOR               0
#define i2c_device_address  0x70
#define HDMI_TVIF_TEST	


#define turn_on_backlight() imapx_gpio_setcfg(__imapx_name_to_gpio(CONFIG_IG_LCD_BACKLIGHT), IG_CTRL0, IG_NORMAL)

#define turn_off_backlight()	\
({				\
	imapx_gpio_setcfg(__imapx_name_to_gpio(CONFIG_IG_LCD_BACKLIGHT), IG_OUTPUT, IG_NORMAL);	\
	imapx_gpio_setpin(__imapx_name_to_gpio(CONFIG_IG_LCD_BACKLIGHT), 0, IG_NORMAL);	\
})

extern unsigned char DoEdidRead (void);

static unsigned short normal_i2c[] = {0, I2C_CLIENT_END};

static struct class *HDMI_class;

I2C_CLIENT_INSMOD;

wait_queue_head_t       HDMI_wait;
spinlock_t              lock;
struct completion	Monitor_Wait;
unsigned int	HDMI_MENU_SWITCH = -1; 
unsigned int	HDMI_HOTPLUG_IN = -1;
unsigned int    HDMI_HOTPLUG_RESULT = 0;
unsigned int VIDEO_MODE = 0;
EXTERL_SYMBOL(VIDEO_MODE);
unsigned int DOUBLE_DISPLAY_MODE = 0;
EXTERL_SYMBOL(DOUBLE_DISPLAY_MODE);
unsigned int PANAUISION_MODE = 0;
EXTERL_SYMBOL(PANAUISION_MODE);
unsigned int HDMI_QUERY_MONITOR_FLAG = -1;
static unsigned int previous_mode = 1;

#define HDMI_HP_POLL_DELAY	1000
extern void imapbl_lowlevel_blctrl(int);
extern void imap_iokey_spken(int);

static const struct i2c_device_id HDMI_i2c_id[] = {
	{ "imap_HDMI", 0 },
	{ }
};

static unsigned char HDMI_V_MODE[7] = {16,4,3,2,18,17,1};

static int HDMI_power(int en);
/*
 *This variable save the HDMI mode supported by the connected TV
 *
 *BIT0:1080P
 *BIT1:720P
 *BIT2:480P_16_9
 *BIT3:480P_4_3
 *BIT4:576P_16_9
 *BIT5:576P_4_3
 *BIT6:640_480
 */
static unsigned char TV_V_MODE;

static struct i2c_client gHDMIClient;
static struct mutex HDMI_lock;

struct imap_HDMI_info {
	struct device *dev;
	struct i2c_client *client;
	struct delayed_work    delayed_work;
};

struct_lcd_timing_param ids_timing[10] = 
{
	{0}, /* this row is removed, lcd timing will be provied by lcd driver, apr.20.2011 */

	{1,0,3,12,0,36,1080,4,5,1,127,1920,109,44,0x06,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,3,12,0,20,720,5,5,0,127,1280,203,40,0x06,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{2,0,3,12,0,30,480,9,6,0,60,720,16,62,0x06,0,0,0,1,1,0,0,0,0,0,0,0,1},
	{2,0,3,12,0,30,480,9,6,0,60,720,16,62,0x06,0,0,0,1,1,0,0,0,0,0,0,0,1},
	{2,0,3,12,0,39,576,5,5,0,68,720,12,64,0x06,0,0,0,1,1,0,0,0,0,0,0,0,1},
	{2,0,3,12,0,39,576,5,5,0,68,720,12,64,0x06,0,0,0,1,1,0,0,0,0,0,0,0,1}, 
	{2,0,3,12,0,33,480,10,2,0,48,640,16,96,0x06,0,0,0,1,1,0,0,0,0,0,0,0,1},    
	{1,0,3,12,0,23,600,1,4,0,88,800,40,128,0x06,0,0,0,1,1,0,0,0,0,0,0,0,1},
	{2,0,3,12,0,29,768,3,6,0,160,1024,24,136,0x06,0,0,0,1,1,0,0,0,0,0,0,0,1},
};

struct_tvif_timing_param tvif_timing[10] = 
{
	{1,0,0,1,0,/**/0,1,0,1,0,0,0,0,0,0,0,/**/0,1,0,0,0,0,
	 /**/18,/**/600,/**/10,/**/18,/**/600,/**/10,/**/408,/**/1024,/**/0,134,32,/**/134,4667,/**/134,4667,/**/1023,/**/599},
	{1,0,0,1,1,/**/0,1,0,1,0,0,1,1,0,0,0,/**/0,1,0,0,0,0,
	 /**/41,/**/1080,/**/4,/**/41,/**/1080,/**/4,/**/272,/**/1920,/**/0,84,44,/**/84,8191,/**/84,8191,/**/1919,/**/1079},
	{1,0,0,1,1,/**/0,1,0,1,0,0,1,1,0,0,0,/**/0,1,0,0,0,0,
	 /**/25,/**/720,/**/5,/**/25,/**/720,/**/5,/**/362,/**/1280,/**/0,106,40,/**/106,8191,/**/106,8191,/**/1279,/**/719},
	{1,0,0,1,2,/**/0,1,0,1,0,0,0,0,0,0,0,/**/0,1,0,0,0,0,
	 /**/36,/**/480,/**/9,/**/36,/**/480,/**/9,/**/130,/**/720,/**/0,12,62,/**/12,5147,/**/12,5147,/**/719,/**/479},
	{1,0,0,1,2,/**/0,1,0,1,0,0,0,0,0,0,0,/**/0,1,0,0,0,0,
	 /**/36,/**/480,/**/9,/**/36,/**/480,/**/9,/**/130,/**/720,/**/0,12,62,/**/12,5147,/**/12,5147,/**/719,/**/479},
	{1,0,0,1,2,/**/0,1,0,1,0,0,0,0,0,0,0,/**/0,1,0,0,0,0,
	 /**/44,/**/576,/**/5,/**/44,/**/576,/**/5,/**/136,/**/720,/**/0,8,64,/**/8,4319,/**/8,4319,/**/719,/**/575},
	{1,0,0,1,2,/**/0,1,0,1,0,0,0,0,0,0,0,/**/0,1,0,0,0,0,
	 /**/44,/**/576,/**/5,/**/44,/**/576,/**/5,/**/136,/**/720,/**/0,8,64,/**/8,4319,/**/8,4319,/**/719,/**/575},
	{1,0,0,1,2,/**/0,1,0,1,0,0,0,0,0,0,0,/**/0,1,0,0,0,0,
	 /**/35,/**/480,/**/10,/**/35,/**/480,/**/10,/**/152,/**/640,/**/0,12,96,/**/12,1599,/**/12,1599,/**/639,/**/479},
	{1,0,0,1,1,/**/0,1,0,1,0,0,0,0,0,0,0,/**/0,1,0,0,0,0,
	 /**/27,/**/600,/**/1,/**/27,/**/600,/**/1,/**/248,/**/800,/**/0,40,128,/**/40,3168,/**/40,3168,/**/799,/**/599},
	{1,0,0,1,2,/**/0,1,0,1,0,0,0,0,0,0,0,/**/0,1,0,0,0,0,
	 /**/35,/**/768,/**/3,/**/35,/**/768,/**/3,/**/312,/**/1024,/**/0,24,136,/**/24,6720,/**/24,6720,/**/1023,/**/767},
};

struct_lds_clk_param ids_clk[10] = 
{
	{0x13,0 /* this value is removed, invoke lcd driver interface instead, apr.20.2011 */},
	{0x24,0x0909},
	{0x24,0x1515},
	{0x1A,0x1D1D},
	{0x1A,0x1D1D},
	{0x1A,0x1D1D},
	{0x1A,0x1D1D},
	{0x18,0x1D1D},
	{0x13,0x1515},
	{0x40,0x1d1d},
};

static unsigned int LCDCON_DataBuf[6];
static unsigned int LCDINT_DataBuf[3];
static unsigned int OVLCON0_DataBuf[12];
static unsigned int OVLCON1_DataBuf[12];
static unsigned int OVLCON2_DataBuf[12];
static unsigned int OVLCON3_DataBuf[10];
static unsigned int OVLCOLCON_DataBuf[14];
static unsigned int OVLPAL0_DataBuf[256];
static unsigned int OVLPAL1_DataBuf[256];
static unsigned int OVLPAL2_DataBuf[128];
static unsigned int OVLPAL3_DataBuf[128];

/*only for system UI*/
void osd_excursion(LCD_TIMING timing)
{
	unsigned int lefttopx;
	unsigned int lefttopy;
	unsigned int rightbottomx;
	unsigned int rightbottomy;
	unsigned int excursionx = 0;
	unsigned int excursiony = 0;

	switch(timing)
	{
		case HDMI_1080P:                                 
			excursionx = (1920 - imapfb_fimd.xres)/2;
			excursiony = (1080 - imapfb_fimd.yres)/2;
			break;                                   

		case HDMI_720P:                                  
			excursionx = (1280- imapfb_fimd.xres)/2; 
			excursiony = (720- imapfb_fimd.yres)/2;  
			break;                                   

		case HDMI_480P_16_9:                             
		case HDMI_480P_4_3:                              
			excursionx = (720 - imapfb_fimd.xres)/2; 
			excursiony = (480 - imapfb_fimd.yres)/2; 
			break;                                   

		case HDMI_576P_16_9:                             
		case HDMI_576P_4_3:                              
			excursionx = (720 - imapfb_fimd.xres)/2; 
			excursiony = (576 - imapfb_fimd.yres)/2; 
			break;                                   

		case HDMI_640_480:                               
			excursionx = (640- imapfb_fimd.xres)/2;  
			excursiony = (480- imapfb_fimd.yres)/2;  
			break;                                   

		case HDMI_800_600:
			excursionx = (800 - imapfb_fimd.xres)/2;
			excursiony = (600 - imapfb_fimd.yres)/2;
			break;

		case HDMI_1024_768:
			excursionx = (1024 - imapfb_fimd.xres)/2;
			excursiony = (768 - imapfb_fimd.yres)/2;
			break;
		default:
			printk("Not supported by now Maybe later\n");
			break;
	}

	lefttopx = 0;//((__raw_readl(IMAP_OVCW0PCAR) >> 16) & (0x7ff));
	lefttopy = 0;//((__raw_readl(IMAP_OVCW0PCAR) >> 0) & (0x7ff));
	rightbottomx = imapfb_fimd.xres;//((__raw_readl(IMAP_OVCW0PCBR) >> 16) & (0x7ff));
	rightbottomy = imapfb_fimd.yres;//((__raw_readl(IMAP_OVCW0PCBR) >> 0) & (0x7ff));
	__raw_writel(IMAP_OVCWxPCAR_LEFTTOPX(lefttopx + excursionx) | IMAP_OVCWxPCAR_LEFTTOPY(lefttopy + excursiony), IMAP_OVCW0PCAR);
	__raw_writel(IMAP_OVCWxPCBR_RIGHTBOTX(rightbottomx + excursionx - 1) | IMAP_OVCWxPCBR_RIGHTBOTY(rightbottomy + excursiony - 1), IMAP_OVCW0PCBR);
}

BOOL lcd_change_timing(LCD_TIMING timing, BOOL tv_IF)
{
	unsigned int temp;
	unsigned char temp_reg[4];
	int i;

/*close lcd screen*/
//	if(timing != LCD)
//	  imapbl_lowlevel_blctrl(0);

	/*Save IDS register value before software reset*/
	/*LCD controller*/
	LCDCON_DataBuf[0] = __raw_readl(IMAP_LCDCON1);
	LCDCON_DataBuf[1] = __raw_readl(IMAP_LCDCON2);
	LCDCON_DataBuf[2] = __raw_readl(IMAP_LCDCON3);
	LCDCON_DataBuf[3] = __raw_readl(IMAP_LCDCON4);
	LCDCON_DataBuf[4] = __raw_readl(IMAP_LCDCON5);
	LCDCON_DataBuf[5] = __raw_readl(IMAP_LCDVCLKFSR);

/*Interrupt */	
	LCDINT_DataBuf[0] = __raw_readl(IMAP_IDSINTPND);
	LCDINT_DataBuf[1] = __raw_readl(IMAP_IDSSRCPND);
	LCDINT_DataBuf[2] = __raw_readl(IMAP_IDSINTMSK);

/*Overlay controller 0*/	
	OVLCON0_DataBuf[0] = __raw_readl(IMAP_OVCDCR);
	OVLCON0_DataBuf[1] = __raw_readl(IMAP_OVCPCR);	
	OVLCON0_DataBuf[2] = __raw_readl(IMAP_OVCBKCOLOR);
	OVLCON0_DataBuf[3] = __raw_readl(IMAP_OVCW0CR);
	OVLCON0_DataBuf[4] = __raw_readl(IMAP_OVCW0PCAR);
	OVLCON0_DataBuf[5] = __raw_readl(IMAP_OVCW0PCBR);
	OVLCON0_DataBuf[6] = __raw_readl(IMAP_OVCW0B0SAR);
	OVLCON0_DataBuf[7] = __raw_readl(IMAP_OVCW0B1SAR);
	OVLCON0_DataBuf[8] = __raw_readl(IMAP_OVCW0VSSR);
	OVLCON0_DataBuf[9] = __raw_readl(IMAP_OVCW0CMR);
	OVLCON0_DataBuf[10] = __raw_readl(IMAP_OVCW0B2SAR);
	OVLCON0_DataBuf[11] = __raw_readl(IMAP_OVCW0B3SAR);

/*Overlay controller 1*/	
	OVLCON1_DataBuf[0] = __raw_readl(IMAP_OVCW1CR);
	OVLCON1_DataBuf[1] = __raw_readl(IMAP_OVCW1PCAR);
	OVLCON1_DataBuf[2] = __raw_readl(IMAP_OVCW1PCBR);
	OVLCON1_DataBuf[3] = __raw_readl(IMAP_OVCW1PCCR);
	OVLCON1_DataBuf[4] = __raw_readl(IMAP_OVCW1B0SAR);
	OVLCON1_DataBuf[5] = __raw_readl(IMAP_OVCW1B1SAR);
	OVLCON1_DataBuf[6] = __raw_readl(IMAP_OVCW1VSSR);
	OVLCON1_DataBuf[7] = __raw_readl(IMAP_OVCW1CKCR);
	OVLCON1_DataBuf[8] = __raw_readl(IMAP_OVCW1CKR);
	OVLCON1_DataBuf[9] = __raw_readl(IMAP_OVCW1CMR);
	OVLCON1_DataBuf[10] = __raw_readl(IMAP_OVCW1B2SAR);
	OVLCON1_DataBuf[11] = __raw_readl(IMAP_OVCW1B3SAR);

/*Overlay controller 2*/	
	OVLCON2_DataBuf[0] = __raw_readl(IMAP_OVCW2CR);
	OVLCON2_DataBuf[1] = __raw_readl(IMAP_OVCW2PCAR);
	OVLCON2_DataBuf[2] = __raw_readl(IMAP_OVCW2PCBR);
	OVLCON2_DataBuf[3] = __raw_readl(IMAP_OVCW2PCCR);
	OVLCON2_DataBuf[4] = __raw_readl(IMAP_OVCW2B0SAR);
	OVLCON2_DataBuf[5] = __raw_readl(IMAP_OVCW2B1SAR);
	OVLCON2_DataBuf[6] = __raw_readl(IMAP_OVCW2VSSR);
	OVLCON2_DataBuf[7] = __raw_readl(IMAP_OVCW2CKCR);
	OVLCON2_DataBuf[8] = __raw_readl(IMAP_OVCW2CKR);
	OVLCON2_DataBuf[9] = __raw_readl(IMAP_OVCW2CMR);
	OVLCON2_DataBuf[10] = __raw_readl(IMAP_OVCW2B2SAR);
	OVLCON2_DataBuf[11] = __raw_readl(IMAP_OVCW2B3SAR);

/*Overlay controller 3*/	
	OVLCON3_DataBuf[0] = __raw_readl(IMAP_OVCW3CR);
	OVLCON3_DataBuf[1] = __raw_readl(IMAP_OVCW3PCAR);
	OVLCON3_DataBuf[2] = __raw_readl(IMAP_OVCW3PCBR);
	OVLCON3_DataBuf[3] = __raw_readl(IMAP_OVCW3PCCR);
	OVLCON3_DataBuf[4] = __raw_readl(IMAP_OVCW3BSAR);
	OVLCON3_DataBuf[5] = __raw_readl(IMAP_OVCW3VSSR);
	OVLCON3_DataBuf[6] = __raw_readl(IMAP_OVCW3CKCR);
	OVLCON3_DataBuf[7] = __raw_readl(IMAP_OVCW3CKR);
	OVLCON3_DataBuf[8] = __raw_readl(IMAP_OVCW3CMR);
	OVLCON3_DataBuf[9] = __raw_readl(IMAP_OVCW3SABSAR);

/*Overlay color controller*/	
	OVLCOLCON_DataBuf[0] = __raw_readl(IMAP_OVCBRB0SAR);
	OVLCOLCON_DataBuf[1] = __raw_readl(IMAP_OVCBRB1SAR);
	OVLCOLCON_DataBuf[2] = __raw_readl(IMAP_OVCOEF11);
	OVLCOLCON_DataBuf[3] = __raw_readl(IMAP_OVCOEF12);
	OVLCOLCON_DataBuf[4] = __raw_readl(IMAP_OVCOEF13);
	OVLCOLCON_DataBuf[5] = __raw_readl(IMAP_OVCOEF21);
	OVLCOLCON_DataBuf[6] = __raw_readl(IMAP_OVCOEF22);
	OVLCOLCON_DataBuf[7] = __raw_readl(IMAP_OVCOEF23);
	OVLCOLCON_DataBuf[8] = __raw_readl(IMAP_OVCOEF31);
	OVLCOLCON_DataBuf[9] = __raw_readl(IMAP_OVCOEF32);
	OVLCOLCON_DataBuf[10] = __raw_readl(IMAP_OVCOEF33);
	OVLCOLCON_DataBuf[11] = __raw_readl(IMAP_OVCOMC);
	OVLCOLCON_DataBuf[12] = __raw_readl(IMAP_OVCBRB2SAR);
	OVLCOLCON_DataBuf[13] = __raw_readl(IMAP_OVCBRB3SAR);

/*Overlay PAL0 */
	for(i=0;i<256;i++)	
		OVLPAL0_DataBuf[i] = __raw_readl(IMAP_OVCW0PAL+i*0x4);

/*Overlay PAL1 */
	for(i=0;i<256;i++)	
		OVLPAL1_DataBuf[i] = __raw_readl(IMAP_OVCW1PAL+i*0x4);

/*Overlay PAL2 */
	for(i=0;i<128;i++)	
		OVLPAL2_DataBuf[i] = __raw_readl(IMAP_OVCW2PAL+i*0x4);

/*Overlay PAL3 */
	for(i=0;i<256;i++)	
		OVLPAL3_DataBuf[i] = __raw_readl(IMAP_OVCW3PAL+i*0x4);

/*Save lcd and osd open status*/
	temp_reg[0] = (OVLCON0_DataBuf[3] & 0x1);
	temp_reg[1]= (OVLCON1_DataBuf[0] & 0x1);
	temp_reg[2]= (OVLCON2_DataBuf[0] & 0x1);
	temp_reg[3]= (OVLCON3_DataBuf[0] & 0x1);

/*LDS module software reset*/
	temp = __raw_readl(rAHBP_RST);
	temp |= 0x1<<6;
	__raw_writel(temp, rAHBP_RST);

	for(i=0;i<10;i++)
		udelay(1000);

	temp = __raw_readl(rAHBP_RST);
	temp &= ~(0x1<<6);
	__raw_writel(temp, rAHBP_RST);
	
/*Config LCD clock*/
	lcd_config_clk(timing);

/*write value back to register*/
	__raw_writel(LCDINT_DataBuf[2], IMAP_IDSINTMSK);

	__raw_writel(OVLCON0_DataBuf[0], IMAP_OVCDCR);
	__raw_writel(OVLCON0_DataBuf[1], IMAP_OVCPCR);
	__raw_writel(OVLCON0_DataBuf[2], IMAP_OVCBKCOLOR);
	__raw_writel((OVLCON0_DataBuf[3] & ~0x1), IMAP_OVCW0CR);
	__raw_writel(OVLCON0_DataBuf[4], IMAP_OVCW0PCAR);
	__raw_writel(OVLCON0_DataBuf[5], IMAP_OVCW0PCBR);
	__raw_writel(OVLCON0_DataBuf[6], IMAP_OVCW0B0SAR);
	__raw_writel(OVLCON0_DataBuf[7], IMAP_OVCW0B1SAR);
	__raw_writel(OVLCON0_DataBuf[8], IMAP_OVCW0VSSR);
	__raw_writel(OVLCON0_DataBuf[9], IMAP_OVCW0CMR);
	__raw_writel(OVLCON0_DataBuf[10], IMAP_OVCW0B2SAR);
	__raw_writel(OVLCON0_DataBuf[11], IMAP_OVCW0B3SAR);
	
	__raw_writel((OVLCON1_DataBuf[0] & ~0x1), IMAP_OVCW1CR);
	__raw_writel(OVLCON1_DataBuf[1], IMAP_OVCW1PCAR);
	__raw_writel(OVLCON1_DataBuf[2], IMAP_OVCW1PCBR);
	__raw_writel(OVLCON1_DataBuf[3], IMAP_OVCW1PCCR);
	__raw_writel(OVLCON1_DataBuf[4], IMAP_OVCW1B0SAR);
	__raw_writel(OVLCON1_DataBuf[5], IMAP_OVCW1B1SAR);
	__raw_writel(OVLCON1_DataBuf[6], IMAP_OVCW1VSSR);
	__raw_writel(OVLCON1_DataBuf[7], IMAP_OVCW1CKCR);
	__raw_writel(OVLCON1_DataBuf[8], IMAP_OVCW1CKR);
	__raw_writel(OVLCON1_DataBuf[9], IMAP_OVCW1CMR);
	__raw_writel(OVLCON1_DataBuf[10], IMAP_OVCW1B2SAR);
	__raw_writel(OVLCON1_DataBuf[11], IMAP_OVCW1B3SAR);
	
	__raw_writel((OVLCON2_DataBuf[0] & ~0x1), IMAP_OVCW2CR);
	__raw_writel(OVLCON2_DataBuf[1], IMAP_OVCW2PCAR);
	__raw_writel(OVLCON2_DataBuf[2], IMAP_OVCW2PCBR);
	__raw_writel(OVLCON2_DataBuf[3], IMAP_OVCW2PCCR);
	__raw_writel(OVLCON2_DataBuf[4], IMAP_OVCW2B0SAR);
	__raw_writel(OVLCON2_DataBuf[5], IMAP_OVCW2B1SAR);
	__raw_writel(OVLCON2_DataBuf[6], IMAP_OVCW2VSSR);
	__raw_writel(OVLCON2_DataBuf[7], IMAP_OVCW2CKCR);
	__raw_writel(OVLCON2_DataBuf[8], IMAP_OVCW2CKR);
	__raw_writel(OVLCON2_DataBuf[9], IMAP_OVCW2CMR);
	__raw_writel(OVLCON2_DataBuf[10], IMAP_OVCW2B2SAR);
	__raw_writel(OVLCON2_DataBuf[11], IMAP_OVCW2B3SAR);

	__raw_writel((OVLCON3_DataBuf[0] & ~0x1), IMAP_OVCW3CR);
	__raw_writel(OVLCON3_DataBuf[1], IMAP_OVCW3PCAR);
	__raw_writel(OVLCON3_DataBuf[2], IMAP_OVCW3PCBR);
	__raw_writel(OVLCON3_DataBuf[3], IMAP_OVCW3PCCR);
	__raw_writel(OVLCON3_DataBuf[4], IMAP_OVCW3BSAR);
	__raw_writel(OVLCON3_DataBuf[5], IMAP_OVCW3VSSR);
	__raw_writel(OVLCON3_DataBuf[6], IMAP_OVCW3CKCR);
	__raw_writel(OVLCON3_DataBuf[7], IMAP_OVCW3CKR);
	__raw_writel(OVLCON3_DataBuf[8], IMAP_OVCW3CMR);
	__raw_writel(OVLCON3_DataBuf[9], IMAP_OVCW3SABSAR);

	__raw_writel(OVLCOLCON_DataBuf[0], IMAP_OVCBRB0SAR);
	__raw_writel(OVLCOLCON_DataBuf[1], IMAP_OVCBRB1SAR);
	__raw_writel(OVLCOLCON_DataBuf[2], IMAP_OVCOEF11);
	__raw_writel(OVLCOLCON_DataBuf[3], IMAP_OVCOEF12);
	__raw_writel(OVLCOLCON_DataBuf[4], IMAP_OVCOEF13);
	__raw_writel(OVLCOLCON_DataBuf[5], IMAP_OVCOEF21);
	__raw_writel(OVLCOLCON_DataBuf[6], IMAP_OVCOEF22);
	__raw_writel(OVLCOLCON_DataBuf[7], IMAP_OVCOEF23);
	__raw_writel(OVLCOLCON_DataBuf[8], IMAP_OVCOEF31);
	__raw_writel(OVLCOLCON_DataBuf[9], IMAP_OVCOEF32);
	__raw_writel(OVLCOLCON_DataBuf[10], IMAP_OVCOEF33);
	__raw_writel(OVLCOLCON_DataBuf[11], IMAP_OVCOMC);
	__raw_writel(OVLCOLCON_DataBuf[12], IMAP_OVCBRB2SAR);
	__raw_writel(OVLCOLCON_DataBuf[13], IMAP_OVCBRB3SAR);

	for(i=0;i<256;i++)
		__raw_writel(OVLPAL0_DataBuf[i], IMAP_OVCW0PAL + 0x4*i);
	
	for(i=0;i<256;i++)
		__raw_writel(OVLPAL1_DataBuf[i], IMAP_OVCW1PAL + 0x4*i);
	
	for(i=0;i<128;i++)
		__raw_writel(OVLPAL2_DataBuf[i], IMAP_OVCW2PAL + 0x4*i);
	
	for(i=0;i<128;i++)
		__raw_writel(OVLPAL3_DataBuf[i], IMAP_OVCW3PAL + 0x4*i);

/*change osd window smaller than Hactive and Vactive back to system resolution if timing 12*/

	if(timing == LCD || (imapfb_fimd.xres == 800 && timing == HDMI_800_600) || (imapfb_fimd.xres == 1024 && timing == HDMI_1024_768))
	{
		/* if LCD use imapfb_fimd configurations, apr.20.2011 */
		__raw_writel(IMAP_OVCWxPCAR_LEFTTOPX(0) | IMAP_OVCWxPCAR_LEFTTOPY(0), IMAP_OVCW0PCAR);
		__raw_writel(IMAP_OVCWxPCBR_RIGHTBOTX(imapfb_fimd.xres - 1) |
		   IMAP_OVCWxPCBR_RIGHTBOTY(imapfb_fimd.yres - 1), IMAP_OVCW0PCBR);

	}
	else
	{
		if(ids_timing[timing].HACTIVE < imapfb_fimd.xres)
		{	
			__raw_writel(IMAP_OVCWxPCAR_LEFTTOPX(0) | IMAP_OVCWxPCAR_LEFTTOPY(0), IMAP_OVCW0PCAR);
			__raw_writel(IMAP_OVCWxPCBR_RIGHTBOTX(ids_timing[timing].HACTIVE -1) | IMAP_OVCWxPCBR_RIGHTBOTY((__raw_readl(IMAP_OVCW0PCBR) & 0x7ff ) -1), IMAP_OVCW0PCBR);
		}

		if(ids_timing[timing].VACTIVE < imapfb_fimd.yres)
		{	
			__raw_writel(IMAP_OVCWxPCAR_LEFTTOPX(0) | IMAP_OVCWxPCAR_LEFTTOPY(0), IMAP_OVCW0PCAR);
			__raw_writel(IMAP_OVCWxPCBR_RIGHTBOTX(((__raw_readl(IMAP_OVCW0PCBR) & (0x7ff<<16)) >> 16) -1) | IMAP_OVCWxPCBR_RIGHTBOTY(ids_timing[timing].VACTIVE -1), IMAP_OVCW0PCBR);
		}
	}

/*change timing*/

	lcd_config_controller(timing);
	if(tv_IF == 1)
	{
		if(timing == LCD)
		{
			__raw_writel((__raw_readl(IMAP_LCDCON5) & ~(0x3<<11)), IMAP_LCDCON5);
		}
		else
		{
			__raw_writel((__raw_readl(IMAP_LCDCON5) | (0x2<<11)), IMAP_LCDCON5);
			tvif_config_controller(timing);
		}
	}

/*enable lcd data output*/	
	if(temp_reg[0])
		__raw_writel((__raw_readl(IMAP_OVCW0CR ) |IMAP_OVCWxCR_ENWIN_ENABLE), IMAP_OVCW0CR);

	if(temp_reg[1])
		__raw_writel((__raw_readl(IMAP_OVCW1CR ) |IMAP_OVCWxCR_ENWIN_ENABLE), IMAP_OVCW1CR);

	if(temp_reg[2])
		__raw_writel((__raw_readl(IMAP_OVCW2CR ) |IMAP_OVCWxCR_ENWIN_ENABLE), IMAP_OVCW2CR);

	if(temp_reg[3])
		__raw_writel((__raw_readl(IMAP_OVCW3CR ) |IMAP_OVCWxCR_ENWIN_ENABLE), IMAP_OVCW3CR);

	if(tv_IF == 1)
	{
		if(timing == LCD)
		{
			__raw_writel((__raw_readl(IMAP_TVICR) & ~(0x1<<31)), IMAP_TVICR);
			__raw_writel((__raw_readl(IMAP_OVCDCR) & ~(0x3)), IMAP_OVCDCR);
			__raw_writel((__raw_readl(IMAP_LCDCON1) | IMAP_LCDCON1_ENVID_ENABLE), IMAP_LCDCON1);
		}
//		else if (timing == HDMI_800_600 || timing == HDMI_1024_768 || timing == HDMI_720P){
//			__raw_writel((__raw_readl(IMAP_OVCDCR) | (0x1<<1)), IMAP_OVCDCR);
//			msleep(10);
//			__raw_writel((__raw_readl(IMAP_TVICR) | (0x1<<31)), IMAP_TVICR);
//		}
		else{
			__raw_writel((__raw_readl(IMAP_OVCDCR) | (0x1<<1)), IMAP_OVCDCR);
			msleep(10);
			__raw_writel((__raw_readl(IMAP_TVICR) | (0x1<<31)), IMAP_TVICR);
		}
	}
	else
	{
		__raw_writel((__raw_readl(IMAP_LCDCON1) | IMAP_LCDCON1_ENVID_ENABLE), IMAP_LCDCON1);
	}

//	if(timing == LCD || timing == HDMI_800_600 || timing == HDMI_1024_768)
//	{
//		imapbl_lowlevel_blctrl(1);	
//	}
}


void lcd_config_clk(LCD_TIMING timing)
{
	unsigned int temp;

	/* if lcd, use imapfb_set_clk interface, apr.20.2011 */
	if(timing == LCD)
	{
		imapfb_set_clk();
		return ;
	}

	temp = readl(rDPLL_CFG); 
	temp &=~(1<<31);
	writel(temp,rDPLL_CFG);

	temp = readl(rDPLL_CFG); 
	temp = ids_clk[timing].DPLLCFG;
	writel(temp,rDPLL_CFG);

	//enable dpll	
	temp = readl(rDPLL_CFG); 
	temp |=(1<<31);
	writel(temp,rDPLL_CFG);

	/*wait untill dpll is locked*/
	while(!(readl(rPLL_LOCKED) & 0x2));

	temp = readl(rDIV_CFG4);
	temp = ids_clk[timing].DIVCFG4;
	writel(temp,rDIV_CFG4);
}

void tvif_config_controller(LCD_TIMING timing)
{
	unsigned int reg_temp[16];

	reg_temp[0] = (tvif_timing[timing].Clock_enable << 31) |
			(tvif_timing[timing].TV_PCLK_mode << 11) |
			(tvif_timing[timing].Inv_clock << 9) |
			(tvif_timing[timing].clock_sel << 8) |
			(tvif_timing[timing].Clock_div << 0 );

	reg_temp[1] = (tvif_timing[timing].tvif_enable << 31) |
			(tvif_timing[timing].ITU601_656n << 30) |
			(tvif_timing[timing].Bit16ofITU60 << 29) |
			(tvif_timing[timing].Direct_data << 28 ) |
			(tvif_timing[timing].Bitswap << 18 ) |
			(tvif_timing[timing].Data_order << 16) |
			(tvif_timing[timing].Inv_vsync << 13 ) |
			(tvif_timing[timing].Inv_hsync << 12 ) |
			(tvif_timing[timing].Inv_href << 11 ) |
			(tvif_timing[timing].Inv_field << 10) |
			(tvif_timing[timing].Begin_with_EAV << 0);

	reg_temp[2] = (tvif_timing[timing].Matrix_mode << 31 ) |
			(tvif_timing[timing].Passby << 30) |
			(tvif_timing[timing].Inv_MSB_in << 29)|
			(tvif_timing[timing].Inv_MSB_out << 28) |
			(tvif_timing[timing].Matrix_oft_b << 8 ) |
			(tvif_timing[timing].Matrix_oft_a << 0);

	reg_temp[3] = tvif_timing[timing].UBA1_LEN;

	reg_temp[4] = tvif_timing[timing].UNBA_LEN;

	reg_temp[5] = tvif_timing[timing].UNBA2_LEN;

	reg_temp[6] = tvif_timing[timing].LBA1_LEN;

	reg_temp[7] = tvif_timing[timing].LNBA_LEN;

	reg_temp[8] = tvif_timing[timing].LBA2_LEN;

	reg_temp[9] = tvif_timing[timing].BLANK_LEN;

	reg_temp[10] = tvif_timing[timing].VIDEO_LEN;

	reg_temp[11] = (tvif_timing[timing].Hsync_VB1_ctrl << 30)|
			(tvif_timing[timing].Hsync_delay << 16) |
			(tvif_timing[timing].Hsync_extend);

	reg_temp[12] = (tvif_timing[timing].Vsync_delay_upper << 16) |
			(tvif_timing[timing].Vsync_extend_upper);

	reg_temp[13] = (tvif_timing[timing].Vsync_delay_lower << 16) |
			(tvif_timing[timing].Vsync_extend_lower);

	reg_temp[14] = tvif_timing[timing].DISP_XSIZE;

	reg_temp[15] = tvif_timing[timing].DISP_YSIZE;	

	__raw_writel(reg_temp[0], IMAP_TVCCR);
	__raw_writel(reg_temp[1], IMAP_TVICR);
	__raw_writel(reg_temp[2], IMAP_TVCMCR);
	__raw_writel(reg_temp[3], IMAP_TVUBA1);
	__raw_writel(reg_temp[4], IMAP_TVUNBA);
	__raw_writel(reg_temp[5], IMAP_TVUBA2);
	__raw_writel(reg_temp[6], IMAP_TVLBA1);
	__raw_writel(reg_temp[7], IMAP_TVLNBA);
	__raw_writel(reg_temp[8], IMAP_TVLBA2);
	__raw_writel(reg_temp[9], IMAP_TVBLEN);
	__raw_writel(reg_temp[10], IMAP_TVVLEN);
	__raw_writel(reg_temp[11], IMAP_TVHSCR);
	__raw_writel(reg_temp[12], IMAP_TVVSHCR);
	__raw_writel(reg_temp[13], IMAP_TVVSLCR);
	__raw_writel(reg_temp[14], IMAP_TVXSIZE);
	__raw_writel(reg_temp[15], IMAP_TVYSIZE);
}

void lcd_config_controller(LCD_TIMING timing)
{	
	unsigned int reg_temp[5];

	if(timing == LCD)
	{
		/* if LCD , use imapfb_fimd configurations, apr.20.2011 */
		writel(imapfb_fimd.lcdcon1, IMAP_LCDCON1); 
		writel(imapfb_fimd.lcdcon2, IMAP_LCDCON2); 
		writel(imapfb_fimd.lcdcon3, IMAP_LCDCON3); 
		writel(imapfb_fimd.lcdcon4, IMAP_LCDCON4); 
		writel(imapfb_fimd.lcdcon5, IMAP_LCDCON5); 

		return ;
	}

	reg_temp[0] = (ids_timing[timing].VCLK <<	8) |		
				(ids_timing[timing].EACH_FRAME << 7) |		
				(ids_timing[timing].LCD_PANNEL << 5) |		
				(ids_timing[timing].BPP_MODE << 1) |
				(ids_timing[timing].LCD_OUTPUT);

	reg_temp[1] = ((ids_timing[timing].VBPD -1) << 24) |
				(((ids_timing[timing].VACTIVE -1) & ~0x400) <<14) |
				((ids_timing[timing].VFPD -1 ) << 6) |
				((ids_timing[timing].VSPW -1));

	reg_temp[2] = (ids_timing[timing].VACTIVE_HIGHBIT <<31) |
				((ids_timing[timing].HBPD -1) << 19) |
				((ids_timing[timing].HACTIVE -1) << 8) |
				((ids_timing[timing].HFPD -1));

	reg_temp[3] = ((ids_timing[timing].HSPW -1));

	reg_temp[4] = ((ids_timing[timing].COLOR_MODE) << 24)|
				((ids_timing[timing].BPP24BL) << 12)|
				((ids_timing[timing].FRM565) << 11) |
				((ids_timing[timing].INVVCLK) << 10)|
				((ids_timing[timing].INVVLINE) <<9 )|
				((ids_timing[timing].INVVFRAME) <<8)|
				((ids_timing[timing].INVVD) <<7) |
				((ids_timing[timing].INVVDEN) << 6) |
				((ids_timing[timing].INVPWREN) << 5)|
				((ids_timing[timing].INVENDLINE) << 4)|
				((ids_timing[timing].PWREN) << 3) |
				((ids_timing[timing].ENLEND) <<2 ) |
				((ids_timing[timing].BSWP) << 1) |
				((ids_timing[timing].HWSWP));
				
	writel(reg_temp[0],IMAP_LCDCON1);
	writel(reg_temp[1],IMAP_LCDCON2);
	writel(reg_temp[2],IMAP_LCDCON3);
	writel(reg_temp[3],IMAP_LCDCON4);
	writel(reg_temp[4],IMAP_LCDCON5);
}

int HDMI_dev_init(struct i2c_client *client)
{
	unsigned int temp;
	unsigned int ret;

	/* get and config interrupt */
	temp = __imapx_name_to_gpio(CONFIG_HDMI_SI9022_INT);

	if(temp == IMAPX_GPIO_ERROR)
	{
		printk(KERN_ERR "Get SI9022 interrupt failed.\n");
		return -ENOTTY;
	}

	
	//si9022_irq = imapx_gpio_to_irq(temp);
	/* config irq status */
	//imapx_gpio_setirq(temp, FILTER_MAX, IG_FALL, 1);

	TPI_Init();

	/*
	ret = ReadByteTPI(TPI_INTERRUPT_STATUS_REG);
	if(!((ret & HOT_PLUG_STATE) >> 2))
	{
		TxPowerState(TX_POWER_STATE_D3);
	}
	*/

	/*
	msleep(100);
	lcd_change_timing(HDMI_1080P);
	msleep(200);
	OnHdmiCableConnected(HDMI_1080P);
	ReadModifyWriteIndexedRegister(INDEXED_PAGE_0, 0x0A, 0x08, 0x08);				
	*/

	return 0;
}

unsigned int hdmi_count = 0;

static void hp_delayed_work(struct work_struct *work)
{
	unsigned int ret;
	struct imap_HDMI_info *HDMI_info =
		        container_of(to_delayed_work(work), struct imap_HDMI_info, delayed_work);
	
	ret = ReadByteTPI(TPI_INTERRUPT_STATUS_REG);

//	printk("ret %d HDMI_QUERY_MONITOR_FLAG %d \
			VIDEO_MODE %d DOUBLE_DISPLAY_MODE %d HDMI_HOTPLUG_RESULT %d \
			hdmi_count %d\n",ret, HDMI_QUERY_MONITOR_FLAG, VIDEO_MODE,\
		       				DOUBLE_DISPLAY_MODE, HDMI_HOTPLUG_RESULT, hdmi_count);
	if(((ret & HOT_PLUG_STATE) >> 2) && \
		(HDMI_QUERY_MONITOR_FLAG == 1) && \
		(VIDEO_MODE == 0 || DOUBLE_DISPLAY_MODE == 0 || PANAUISION_MODE == 0) && \
	       	HDMI_HOTPLUG_RESULT == 0 && \
		hdmi_count >5)
	{
		HDMI_QUERY_MONITOR_FLAG = -1;
		HDMI_HOTPLUG_IN = 1;
		HDMI_HOTPLUG_RESULT = 1;
		hdmi_count = 0;
		complete(&Monitor_Wait);
	}
	else if((!((ret & HOT_PLUG_STATE) >> 2)) && \
		(HDMI_QUERY_MONITOR_FLAG == 1) && \
		(VIDEO_MODE == 1 || DOUBLE_DISPLAY_MODE == 1 || PANAUISION_MODE == 1) && \
		HDMI_HOTPLUG_RESULT == 1 &&\
		hdmi_count >5)
	{
		HDMI_QUERY_MONITOR_FLAG = -1;
		HDMI_HOTPLUG_IN= 0;
		HDMI_HOTPLUG_RESULT = 0;
		hdmi_count = 0;
		complete(&Monitor_Wait);
	}

	hdmi_count ++ ;
sche:
	schedule_delayed_work(&HDMI_info->delayed_work,
			msecs_to_jiffies(HDMI_HP_POLL_DELAY));
}



static int HDMI_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct device *dev = NULL;
	struct task_struct *t;

	printk(KERN_INFO "Device for HDMI will be Initializad\n");

	/* register this i2c device with the driver core */
	dev = device_create(HDMI_class, NULL, MKDEV(HDMI_MAJOR, HDMI_MINOR), NULL, "HDMI");
	if (IS_ERR(dev))
	{
		ret = PTR_ERR(dev);
		goto exit;
	}

	dev = device_create(HDMI_class, NULL, MKDEV(HDMI_POLL_MAJOR, HDMI_MINOR), NULL, "HDMI_poll");
	if (IS_ERR(dev))
	{
		ret = PTR_ERR(dev);
		goto exit;
	}

	memcpy(&gHDMIClient, client, sizeof(struct i2c_client));

	mutex_init(&HDMI_lock);

	/* HDMI device initialization */
	ret = HDMI_dev_init(client);
	if (ret < 0)
	{
		printk(KERN_ERR "HDMI_i2c_probe: failed to initialise SI9022\n");
		goto exit;
	}

	printk(KERN_INFO "Init HDMI device OK\n");

	return 0;

exit:
	return ret;
}

static int HDMI_i2c_remove(struct i2c_client *client)
{
	printk(KERN_INFO "Remove HDMI device driver\n");

	mutex_destroy(&HDMI_lock);

	return 0;
}

static unsigned int 
HDMI_poll(struct file *file, struct poll_table_struct *wait)
{
	unsigned int mask;

	poll_wait(file, &HDMI_wait, wait);

	if(HDMI_HOTPLUG_IN == 1)
		mask |= POLLIN | POLLRDNORM;
	else
		mask = POLLERR;

	return mask;
}

static int HDMI_open(struct inode *inode, struct file *file)
{
	file->private_data = &gHDMIClient;
	return 0;
}

static int HDMI_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static ssize_t
HDMI_poll_write(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	HDMI_QUERY_MONITOR_FLAG = 1;
	wait_for_completion(&Monitor_Wait);
	printk("Monitor wait completion\n");

	return 0;
}

static ssize_t
HDMI_poll_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	unsigned int HDMI_HOTPLUG_TYPE = 0;
	int ret = 0;

	if(HDMI_HOTPLUG_IN == -1)
	{
		if(HDMI_MENU_SWITCH == -1)
			HDMI_HOTPLUG_TYPE = 1;	/*No hotplug	No Menu button pressed*/
		else if(HDMI_MENU_SWITCH == 0)
			HDMI_HOTPLUG_TYPE = 2;	/*No hotplug	Menu Button Pressed to LCD*/
		else if(HDMI_MENU_SWITCH == 1)
			HDMI_HOTPLUG_TYPE = 3;	/*No hotplug	Menu Button Pressed to TV*/
	}
	else if(HDMI_HOTPLUG_IN == 0)
	{
		if(HDMI_MENU_SWITCH == -1)
			HDMI_HOTPLUG_TYPE = 4;	/*hotplug out detected No menu button pressed*/
		else if (HDMI_MENU_SWITCH == 0)
			HDMI_HOTPLUG_TYPE = 5 ;	/*hotplug out detected menu button pressed to LCD*/
		else if (HDMI_MENU_SWITCH == 1)
			HDMI_HOTPLUG_TYPE = 6 ;	/*hotplug out detected menu button pressed to TV*/
	}
	else if(HDMI_HOTPLUG_IN == 1)
	{
		if(HDMI_MENU_SWITCH == -1)
			HDMI_HOTPLUG_TYPE = 7;	/*hotplug in detected No menu button pressed*/
		else if (HDMI_MENU_SWITCH == 0)
			HDMI_HOTPLUG_TYPE = 8 ;	/*hotplug in detected menu button pressed to LCD*/
		else if (HDMI_MENU_SWITCH == 1)
			HDMI_HOTPLUG_TYPE = 9 ;	/*hotplug in detected menu button pressed to TV*/
	}

	if(copy_to_user((void __user *)buf, &HDMI_HOTPLUG_TYPE, sizeof(unsigned int)))
	{
		printk(KERN_ERR "[HDMI_ioctl]: copy to user space error\n");
		ret = -EFAULT;
	}
	HDMI_HOTPLUG_IN = -1;
	HDMI_MENU_SWITCH = -1;

	return ret;
}

static long HDMI_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	unsigned int temp1;
	unsigned int temp2;
	unsigned int check_result;
	unsigned long flags;
	void __user *argp = (void __user *)arg;
	LCD_TIMING HDMI_TIMING_TYPE ;
	
	mutex_lock(&HDMI_lock);

	switch(cmd)
	{
		case HDMI_CHECK_MODE:
			printk("%s commond :HDMI_CHECK previous_mode %d\n",__func__, previous_mode);
			if(copy_to_user((void __user *)argp, &previous_mode, sizeof(unsigned int)))
			{
				printk(KERN_ERR "[HDMI_ioctl]: copy to user space error\n");
				ret = -EFAULT;
			}
			break;
		case HDMI_CHECK_CONNECT:
			printk("%s commond :HDMI_CHECK_CONNECT\n",__func__);
			if(copy_to_user((void __user *)argp, &HDMI_HOTPLUG_RESULT, sizeof(unsigned int)))
			{
				printk(KERN_ERR "[HDMI_ioctl]: copy to user space error\n");
				ret = -EFAULT;
			}
			break;
		case HDMI_SET_MODE:
			printk("%s commond :HDMI_SET_MODE previous_mode %d\n",__func__, previous_mode);
			if(copy_from_user(&previous_mode, argp, sizeof(unsigned int)))
			{
				printk(KERN_ERR "[HDMI_ioctl]: copy previous_mode from  user space error\n");
				ret = -EFAULT;
			}
			break;
		case HDMI_SET_NOTMAL_TIMING:  //return
			if(VIDEO_MODE == 1 || DOUBLE_DISPLAY_MODE == 1 || PANAUISION_MODE == 1)
			{
				printk("%s commond : HDMI_SET_NOTMAL_TIMING\n",__func__);
#ifdef CONFIG_UI_MAP_COLOR
				if(VIDEO_MODE == 1)
				{
					__raw_writel(~0x1<<24, IMAP_OVCW0CMR);
					__raw_writel(~0x1<<24, IMAP_OVCW3CMR);
				}
#endif
				turn_off_backlight();
				VIDEO_MODE = 0;
				DOUBLE_DISPLAY_MODE = 0;
				PANAUISION_MODE = 0;
				DisableTMDS();
				msleep(1000);
				lcd_change_timing(LCD, 1);
				imap_iokey_spken(1);
				turn_on_backlight();
			}
			break;
		case HDMI_SET_VIDEO_TIMING:  //enter hdmi mode
			if(copy_from_user(&HDMI_TIMING_TYPE, argp, sizeof(unsigned int)))
			{
				printk(KERN_ERR "[HDMI_ioctl]: copy from user space error\n");
				ret = -EFAULT;
			}
			if(VIDEO_MODE == 0)
			{
				turn_off_backlight();
				imap_iokey_spken(0);
				printk("%s commond : HDMI_SET_VIDEO_TIMING\n",__func__);
#ifdef CONFIG_UI_MAP_COLOR
				__raw_writel(0x1<<24 | ((0x0 & 0xffffff)<<0), IMAP_OVCW0CMR);
				__raw_writel(0x1<<24 | ((0x0 & 0xffffff)<<0), IMAP_OVCW3CMR);
#endif
				lcd_change_timing(HDMI_TIMING_TYPE - HDMI_640_480, 1);
				msleep(50);
				OnHdmiCableConnected(HDMI_TIMING_TYPE - HDMI_640_480);
				ReadModifyWriteIndexedRegister(INDEXED_PAGE_0, 0x0A, 0x08, 0x08);	
				VIDEO_MODE = 1;
				DOUBLE_DISPLAY_MODE = 0;
				PANAUISION_MODE = 0;
			}
			break;
		case HDMI_SET_DOUBLE_DISPLAY_TIMING:
#ifdef CONFIG_DOUBLE_DISPLAY_MODE
#ifdef CONFIG_UI_MAP_COLOR
			if(VIDEO_MODE == 1)
			{
				__raw_writel(~0x1<<24, IMAP_OVCW0CMR);
				__raw_writel(~0x1<<24, IMAP_OVCW3CMR);
			}
#endif
			if(DOUBLE_DISPLAY_MODE == 0)
			{
				turn_off_backlight();
				DisableTMDS();
				printk("%s commond :HDMI_SET_DOUBLE_DISPLAY_TIMING\n",__func__);
				if(imapfb_fimd.xres == 800){
					lcd_change_timing(HDMI_800_600, 1);
					msleep(50);
					OnHdmiCableConnected(HDMI_800_600);
					ReadModifyWriteIndexedRegister(INDEXED_PAGE_0, 0x0A, 0x08, 0x08);	
				}else if(imapfb_fimd.xres == 1024){
					lcd_change_timing(HDMI_1024_768, 1);
					msleep(50);
					OnHdmiCableConnected(HDMI_1024_768);
					ReadModifyWriteIndexedRegister(INDEXED_PAGE_0, 0x0A, 0x08, 0x08);	
				}
				DOUBLE_DISPLAY_MODE = 1;
				PANAUISION_MODE = 0;
				VIDEO_MODE = 0;
				previous_mode = 1;
				msleep(1000);
				turn_on_backlight();
				imap_iokey_spken(1);
			}
#else
			printk("double display mode not supported\n");
			if(VIDEO_MODE == 1|| PANAUISION_MODE == 1 || DOUBLE_DISPLAY_MODE == 1)
			{
				VIDEO_MODE = 0;
				DOUBLE_DISPLAY_MODE = 0;
				PANAUISION_MODE = 0;
				DisableTMDS();
				lcd_change_timing(LCD, 1);
				imap_iokey_spken(1);
				turn_on_backlight();
			}
			ret = -1;
#endif
			break;
		case HDMI_SET_PANAUISION_TIMING:
#ifdef CONFIG_PANAUISION_MODE
#ifdef CONFIG_UI_MAP_COLOR
			if(VIDEO_MODE == 1)
			{
				__raw_writel(~0x1<<24, IMAP_OVCW0CMR);
				__raw_writel(~0x1<<24, IMAP_OVCW3CMR);
			}
#endif
			if(PANAUISION_MODE == 0)
			{
				turn_off_backlight();
				imap_iokey_spken(0);
				DisableTMDS();
				printk("%s commond :HDMI_PANAUISION_TIMING\n",__func__);
				if(imapfb_fimd.xres == 800){
					lcd_change_timing(HDMI_1024_768, 1);
					osd_excursion(HDMI_1024_768);
					msleep(50);
					OnHdmiCableConnected(HDMI_1024_768);
					ReadModifyWriteIndexedRegister(INDEXED_PAGE_0, 0x0A, 0x08, 0x08);	
				}else if(imapfb_fimd.xres == 1024){
					lcd_change_timing(HDMI_720P, 1);
					osd_excursion(HDMI_720P);
					msleep(50);
					OnHdmiCableConnected(HDMI_720P);
					ReadModifyWriteIndexedRegister(INDEXED_PAGE_0, 0x0A, 0x08, 0x08);	
				}
				PANAUISION_MODE = 1;
				DOUBLE_DISPLAY_MODE = 0;
				VIDEO_MODE = 0;
				previous_mode = 2;
			}
#else
			printk("panauision mode not supported\n");
			if(VIDEO_MODE == 1|| PANAUISION_MODE == 1 || DOUBLE_DISPLAY_MODE == 1)
			{
				VIDEO_MODE = 0;
				DOUBLE_DISPLAY_MODE = 0;
				PANAUISION_MODE = 0;
				DisableTMDS();
				lcd_change_timing(LCD, 1);
				imap_iokey_spken(1);
				turn_on_backlight();
			}
			ret = -1;
#endif
			break;
		case HDMI_TV_SUPPORT_V_MODE:
			if(copy_to_user((void __user *)argp, &TV_V_MODE, sizeof(unsigned int)))
			{
				printk(KERN_ERR "[HDMI_ioctl]: copy to user space error\n");
				ret = -EFAULT;
			}

			break;
		default:
			printk(KERN_ERR "[HDMI_ioctl]: unknown command type\n");
			ret = -EFAULT;
			break;
	}
	mutex_unlock(&HDMI_lock);
	return ret;
}

#if defined(CONFIG_PM)
int HDMI_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	printk(KERN_INFO "iic Suspend SI9022\n");
	if(HDMI_HOTPLUG_RESULT == 1 && (( VIDEO_MODE == 1) || (DOUBLE_DISPLAY_MODE == 1) || (PANAUISION_MODE == 1)))
	{
		TxPowerState(TX_POWER_STATE_D3);
		lcd_change_timing(LCD, 1);
		VIDEO_MODE = 0;
		DOUBLE_DISPLAY_MODE = 0;
		PANAUISION_MODE = 0;
		HDMI_HOTPLUG_RESULT = 0;
	}

	HDMI_power(0);
	return 0;
}
		
int HDMI_i2c_resume(struct i2c_client *client)
{
	printk(KERN_INFO "iic Resume SI9022\n");
	
	HDMI_power(1);
	TPI_Init();

	return 0;
}		
#endif

static const struct file_operations HDMI_fops = {
	.owner                  = THIS_MODULE,
	.unlocked_ioctl = HDMI_ioctl,
	.open                   = HDMI_open,
	.poll		= HDMI_poll,
	.release                        = HDMI_release,
};

static const struct file_operations HDMI_poll_fops = {
	.owner                  = THIS_MODULE,
	.read		= HDMI_poll_read,
	.write		= HDMI_poll_write,
};

static struct i2c_driver HDMI_i2c_driver = {
	.driver = {
		.name = "imap_HDMI-i2c",
		.owner = THIS_MODULE,
	},
	.probe = HDMI_i2c_probe,
	.remove = HDMI_i2c_remove,
#if defined(CONFIG_PM) 
	.suspend = HDMI_i2c_suspend,
	.resume = HDMI_i2c_resume,
#endif
	.id_table = HDMI_i2c_id,
};

static int HDMI_power(int en)
{
	unsigned long gpio;

	gpio = __imapx_name_to_gpio(CONFIG_HDMI_SI9022_POWER);

	if(gpio == IMAPX_GPIO_ERROR)
	  /* power gpio is not configurated */
	  return 0;

	imapx_gpio_setcfg(gpio, IG_OUTPUT, IG_NMSL);
	if(en)
	  imapx_gpio_setpin(gpio, 1, IG_NMSL);
	else
	  imapx_gpio_setpin(gpio, 0, IG_NMSL);

	return 0;
}

static int __init HDMI_probe(struct platform_device *pdev)
{
	struct i2c_board_info info;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct imap_HDMI_info *HDMI_info;
	int ret;

#ifdef CONFIG_FAKE_PM
	if_in_suspend = 0;
#endif

	printk(KERN_INFO "HDMI Probe to add i2c driver for HDMI device\n");

	HDMI_power(1);
	HDMI_info =(struct imap_HDMI_info *) kzalloc(sizeof(struct imap_HDMI_info), GFP_KERNEL);
	if(!HDMI_info)
	{
		printk(KERN_ERR "Cannot allocate for HDMI_info\n");
		return -ENOMEM;
	}

	ret = register_chrdev(HDMI_MAJOR, "imap-HDMI", &HDMI_fops);
	if (ret)
		goto out;

	ret = register_chrdev(HDMI_POLL_MAJOR, "imap-HDMI_poll", &HDMI_poll_fops);
	if (ret)
		goto out;

	HDMI_class = class_create(THIS_MODULE, "HDMI_dev");
	if (IS_ERR(HDMI_class))
	{
		printk(KERN_ERR "HDMI_init: fail to create HDMI device class\n");
		ret = PTR_ERR(HDMI_class);
		goto out_unreg_chrdev;
	}

	/* Add i2c_driver */
	ret = i2c_add_driver(&HDMI_i2c_driver);
	if(ret)
	{
		printk(KERN_ERR "HDMI_init: fail to register i2c driver\n");
		goto out_unreg_class;
	}

	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = i2c_device_address;
	strlcpy(info.type, "imap_HDMI", I2C_NAME_SIZE);

	adapter = i2c_get_adapter(CONFIG_HDMI_SI9022_I2C + 1);
	if (!adapter)
	{
		printk(KERN_ERR "HDMI_init: can't get i2c adapter\n");
		goto err_adapter;
	}

	HDMI_info->client = i2c_new_device(adapter, &info);
	i2c_put_adapter(adapter);
	if (!(HDMI_info->client))
	{
		printk(KERN_ERR "HDMI_init: can't add i2c device at 0x%x\n", (unsigned int)info.addr);
		goto err_adapter;
	}

	spin_lock_init(&lock);
	init_completion(&Monitor_Wait); 
	HDMI_info->dev = &(pdev->dev);

	INIT_DELAYED_WORK(&HDMI_info->delayed_work, hp_delayed_work);

	schedule_delayed_work(&HDMI_info->delayed_work,
	                      msecs_to_jiffies(HDMI_HP_POLL_DELAY));


	platform_set_drvdata(pdev, HDMI_info);

	printk(KERN_INFO "HDMI device add i2c driver OK!\n");

	return 0;

err_adapter:
	i2c_del_driver(&HDMI_i2c_driver);
out_unreg_class:
	class_destroy(HDMI_class);
out_unreg_chrdev:
	unregister_chrdev(HDMI_MAJOR, "imap-HDMI");
	unregister_chrdev(HDMI_POLL_MAJOR, "imap-HDMI_poll");

out:
	printk(KERN_ERR "%s: Driver Initialisation failed\n", __FILE__);

	return ret;
}

static int HDMI_remove(struct platform_device *pdev)
{
	struct imap_HDMI_info *info = (struct imap_HDMI_info *)platform_get_drvdata(pdev);

	i2c_unregister_device(info->client);
	i2c_del_driver(&HDMI_i2c_driver);
	class_destroy(HDMI_class);
	unregister_chrdev(HDMI_MAJOR, "imap-HDMI");
	unregister_chrdev(HDMI_POLL_MAJOR, "imap-HDMI_poll");

	return 0;
}

#if defined(CONFIG_PM)
int HDMI_suspend(struct platform_device *dev, pm_message_t state)
{
	printk(KERN_INFO "Suspend EP932\n");
	return 0;
}
		
int HDMI_resume(struct platform_device *dev)
{
        printk(KERN_INFO "Resume EP932\n");
	return 0;
}		
#endif

static struct platform_driver HDMI_driver = {
	.driver = {
		.name = "imap-HDMI",
		.owner = THIS_MODULE,
	},
	.probe = HDMI_probe,
	.suspend = HDMI_suspend,
	.resume = HDMI_resume,
	.remove = HDMI_remove,
};

static int __init HDMI_init(void)
{
	return platform_driver_register(&HDMI_driver);
}

static void __exit HDMI_exit(void)
{
	platform_driver_unregister(&HDMI_driver);
}

module_init(HDMI_init);
module_exit(HDMI_exit);

MODULE_DESCRIPTION("Infotm HDMI driver");
MODULE_AUTHOR("Alex Zhang, <alex.zhang@infotmic.com.cn>");
MODULE_LICENSE("GPL");
