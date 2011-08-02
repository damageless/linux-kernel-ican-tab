/***************************************************************************** 
 ** hy252_xx.c 
 ** 
 ** Copyright (c) 2009~2014 ShangHai Infotm Ltd all rights reserved. 
 **     
 ** This program is free software; you can redistribute it and/or modify
 ** it under the terms of the GNU General Public License as published by
 ** the Free Software Foundation; either version 2 of the License, or
 ** (at your option) any later version.
 **             
 ** Description: sensor config for sensor of ov2655 production 
 **             
 ** Author:     
 **     neville <haixu_fu@infotm.com>
 **      
 ** Revision History: 
 ** ­­­­­­­­­­­­­­­­­ 
 ** 2.0  11/08/2010 neville   
 *******************************************************************************/



#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#include <plat/clock.h>

#include <mach/imapx_base_reg.h>
#include <mach/irqs.h>
#include "imapx200_cam.h"

#include "hy252_XX.h"
static struct sensor_ops *ops;
#define cam_hy252_read(b, c, d) ops->i2c_read(c, b, 1, d) 
#define cam_hy252_write(b) ops->i2c_write(b, 2)           
                                                                                         
static int HI253_write_cmos_sensor(unsigned char addr, unsigned value)
{
	struct hy252_regval_list  regval;
	regval.reg = addr;
	regval.value  = value;
	cam_hy252_write((unsigned char *)(&regval));
	return 0;
}
static  char HI253_read_cmos_sensor(unsigned char addr)
{
	char buf;
	cam_hy252_read((unsigned char *)(&addr), &buf, 1);
	return buf;
}


int hy252_xx_reset(struct imapx200_camif_param_t *param)
{

	uint32_t tmp = 0;
	printk(KERN_INFO "hy252 cam reset\n");
	writel(0x0, param->ioaddr + IMAP_CIGCTRL);

	tmp = readl(param->ioaddr+IMAP_CIGCTRL);
	tmp |= (0x1 << 1);
	writel(tmp, param->ioaddr+IMAP_CIGCTRL);
	mdelay(100);

	tmp = readl(param->ioaddr+IMAP_CIGCTRL);
	tmp &= ~(0x1 << 1);
	writel(tmp, param->ioaddr+IMAP_CIGCTRL);
	mdelay(100);

	tmp = readl(param->ioaddr+IMAP_CIGCTRL);
	tmp |= (0x1 << 1);
	writel(tmp, param->ioaddr+IMAP_CIGCTRL);

	return 0;
}
/*
 *   init & close sensor 
 */

int hy252_start(void){
	int i,ret; 
	char reg0x04 = 0;
	cam_hy252_read((unsigned char *)(&HY252_04), &reg0x04, 1);
	printk(KERN_ERR "get hy253 ID  %x\n",reg0x04);

	for(i = 0; i < (sizeof(hy252_init_regs) / 2); i++)
	{
		ret = cam_hy252_write( (unsigned char *)(&hy252_init_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	}
	return 0;
}

uint32_t hy252_xx_get_id(void)
{       
	uint32_t cmos_id;       
	cmos_id = HI253_read_cmos_sensor(HY252_04);
	return cmos_id;
}


int hy252_close(void)
{
	int i, ret;
	printk(KERN_INFO "hy252_close\n");	

	for(i = 0; i < ((sizeof(hy252_stop_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char *)(&hy252_stop_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	}
	return 0; 
} 
/*
 *  set capture mode
 */
	/* 
 	 * used to change fps to 22
	 */
int  hy252_switch_low_svga(void)
{

	int i, ret;	

	printk(KERN_INFO "hy252_switch_low_svga\n");	
#if 0
	for(i = 0; i < ((sizeof(hy252_svga_low_regs) / 3)); i++)
	{
		ret = cam_hy252_write( (unsigned char *)(&hy252_svga_low_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	}
#endif
	return 0;
}

	/* 
 	 * used to change fps to 18
	 */
int  hy252_switch_high_svga(void)
{
	int i, ret;

	printk(KERN_INFO "hy252_switch_high_svga\n");	
#if 0
	for(i = 0; i < ((sizeof(hy252_svga_high_regs) / 3)); i++)
	{
		ret = cam_hy252_write( (unsigned char *)(&hy252_svga_high_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	}
#endif
	return 0;
}

 /*
  * 1600x1200
  */
int  hy252_switch_high_xuga(void)
{
	int i, ret;
	printk(KERN_INFO "HI253 to high xuga\n");

//	cam_hy252_write( (unsigned char *)(hy252_before));
//	mdelay(5);
	for(i = 0; i < ((sizeof(hy252_xuga_high_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char *)(&hy252_xuga_high_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	}
	//cam_hy252_write( (unsigned char *)(hy252_after));
	return 0;
}

/*
 * 1280x960
 */
int hy252_switch_upmid_xuga(void)
{
	int i, ret;

	//cam_hy252_write( (unsigned char *)(hy252_before));
	//mdelay(5);
	for(i = 0; i < ((sizeof(hy252_xuga_upmid_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char *)(&hy252_xuga_upmid_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	}
	//cam_hy252_write( (unsigned char *)(hy252_after));
	return 0;
}

/*
 * 640 * 480
 */
int  hy252_switch_mid_xuga(void)
{
	int i, ret;
	printk(KERN_INFO "HI253 to mid xuga\n");
	//cam_hy252_write( (unsigned char *)(hy252_before));
	//mdelay(5);
	for(i = 0; i < ((sizeof(hy252_xuga_mid_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char *)(&hy252_xuga_mid_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	}
	//cam_hy252_write( (unsigned char *)(hy252_after));
	return 0;
}
/*
 * 320 * 240
 */

int  hy252_switch_low_xuga(void)
{
	int i, ret;
	printk(KERN_INFO "HI253  to low xuga\n");
	//cam_hy252_write( (unsigned char *)(hy252_before));
	//mdelay(5);
	for(i = 0; i < ((sizeof(hy252_xuga_low_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char *)(&hy252_xuga_low_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	}
	//cam_hy252_write( (unsigned char *)(hy252_after));
	return 0;
}

/*
 * svga --> xuga
 */

int hy252_svga_to_xuga(int width, int height){
	uint32_t HI253_pv_HI253_exposure_lines = 0x0249f0;
	uint32_t HI253_cp_HI253_exposure_lines = 0;
	uint8_t  HI253_Sleep_Mode;
	uint8_t  CLK_DIV_REG;

	printk(KERN_INFO "Taking Picture!\n");
	if( width <=  640 && height <= 480){
		
		HI253_write_cmos_sensor(0x03, 0x00);
/*
		HI253_write_cmos_sensor(0x20, 0x00);
		HI253_write_cmos_sensor(0x21, 0x04);
		HI253_write_cmos_sensor(0x22, 0x00);
		HI253_write_cmos_sensor(0x23, 0x07);
*/
		HI253_write_cmos_sensor(0x40, 0x01);
		HI253_write_cmos_sensor(0x41, 0x68);
		HI253_write_cmos_sensor(0x42, 0x00);
		HI253_write_cmos_sensor(0x43, 0x14);

#if 0
		HI253_write_cmos_sensor(0x03, 0x10);
		HI253_write_cmos_sensor(0x3f, 0x00);
		//page 12
		HI253_write_cmos_sensor(0x03, 0x12);
		HI253_write_cmos_sensor(0x20, 0x0f);
		HI253_write_cmos_sensor(0x21, 0x0f);
		HI253_write_cmos_sensor(0x90, 0x5d);
		//page 13
		HI253_write_cmos_sensor(0x03, 0x13);
		HI253_write_cmos_sensor(0x80, 0xfd);
#endif
		// 800 * 600
	//	HI253_write_cmos_sensor(0x03, 0x00);
	//	HI253_write_cmos_sensor(0x10, 0x11);

		HI253_write_cmos_sensor(0x03, 0x20);
		HI253_write_cmos_sensor(0x86, 0x01);
		HI253_write_cmos_sensor(0x87, 0xf4);

		HI253_write_cmos_sensor(0x8b, 0x83);
		HI253_write_cmos_sensor(0x8c, 0xd6);
		HI253_write_cmos_sensor(0x8d, 0x6d);
		HI253_write_cmos_sensor(0x8e, 0x60);

		HI253_write_cmos_sensor(0x9c, 0x17);
		HI253_write_cmos_sensor(0x9d, 0x70);
		HI253_write_cmos_sensor(0x9e, 0x01);
		HI253_write_cmos_sensor(0x9f, 0xf4);

		HI253_write_cmos_sensor(0x03, 0x20);
		HI253_pv_HI253_exposure_lines = (HI253_read_cmos_sensor(0x80) << 16)|(HI253_read_cmos_sensor(0x81) << 8)|HI253_read_cmos_sensor(0x82);

		HI253_cp_HI253_exposure_lines=HI253_pv_HI253_exposure_lines;

		if(HI253_cp_HI253_exposure_lines<1)
			HI253_cp_HI253_exposure_lines=1;

		HI253_write_cmos_sensor(0x03, 0x20);
		HI253_write_cmos_sensor(0x83, HI253_cp_HI253_exposure_lines >> 16);
		HI253_write_cmos_sensor(0x84, (HI253_cp_HI253_exposure_lines >> 8) & 0x000000FF);
		HI253_write_cmos_sensor(0x85, HI253_cp_HI253_exposure_lines & 0x000000FF);
	}
	else {

		HI253_write_cmos_sensor(0x03,0x00);
//		HI253_Sleep_Mode = (HI253_read_cmos_sensor(0x01) & 0xfe);
//		HI253_Sleep_Mode |= 0x01;
//		HI253_write_cmos_sensor(0x01, HI253_Sleep_Mode);

		CLK_DIV_REG=(HI253_read_cmos_sensor(0x12)&0xFc);

		//read the shutter (manual exptime)

		HI253_write_cmos_sensor(0x03, 0x20);
		HI253_pv_HI253_exposure_lines = (HI253_read_cmos_sensor(0x80) << 16)|(HI253_read_cmos_sensor(0x81) << 8)|HI253_read_cmos_sensor(0x82);

		HI253_cp_HI253_exposure_lines = HI253_pv_HI253_exposure_lines/2;

		HI253_write_cmos_sensor(0x03, 0x00);

//		HI253_write_cmos_sensor(0x20, 0x00);
//		HI253_write_cmos_sensor(0x21, 0x0a);
//		HI253_write_cmos_sensor(0x22, 0x00);
//		HI253_write_cmos_sensor(0x23, 0x0a);

		HI253_write_cmos_sensor(0x40, 0x01);
		HI253_write_cmos_sensor(0x41, 0x58);// 0168
		HI253_write_cmos_sensor(0x42, 0x00);
		HI253_write_cmos_sensor(0x43, 0x14);

//		HI253_write_cmos_sensor(0x03, 0x10);
//		HI253_write_cmos_sensor(0x3f, 0x00);
		//PAGE 12/ HI253_write

//		HI253_write_cmos_sensor(0x03, 0x12);
//		HI253_write_cmos_sensor(0x20, 0x0f);
//		HI253_write_cmos_sensor(0x21, 0x0f);
//		HI253_write_cmos_sensor(0x90, 0x5d);
		//PAGE 13
//		HI253_write_cmos_sensor(0x03, 0x13);
//		HI253_write_cmos_sensor(0x80, 0xfd);
		//1600 *1200
	//	HI253_write_cmos_sensor(0x03,0x00);
	//	HI253_write_cmos_sensor(0x10,0x00);

		CLK_DIV_REG=CLK_DIV_REG|0x1;
		HI253_write_cmos_sensor(0x03, 0x00);
		HI253_write_cmos_sensor(0x12,CLK_DIV_REG);

		HI253_write_cmos_sensor(0x03, 0x20);
        	HI253_write_cmos_sensor(0x86, 0x01);
		HI253_write_cmos_sensor(0x87, 0xf0);

		HI253_write_cmos_sensor(0x8b, 0x41);
		HI253_write_cmos_sensor(0x8c, 0xe0);
		HI253_write_cmos_sensor(0x8d, 0x36);
		HI253_write_cmos_sensor(0x8e, 0x40);

		HI253_write_cmos_sensor(0x9c, 0x0d);
		HI253_write_cmos_sensor(0x9d, 0x90);
		HI253_write_cmos_sensor(0x9e, 0x01);
		HI253_write_cmos_sensor(0x9f, 0xf0);

		if(HI253_cp_HI253_exposure_lines<1)
			HI253_cp_HI253_exposure_lines=1;

		HI253_write_cmos_sensor(0x03, 0x20);
		HI253_write_cmos_sensor(0x83, HI253_cp_HI253_exposure_lines >> 16);
		HI253_write_cmos_sensor(0x84, (HI253_cp_HI253_exposure_lines >> 8) & 0x000000FF);
		HI253_write_cmos_sensor(0x85, HI253_cp_HI253_exposure_lines & 0x000000FF);

//		HI253_write_cmos_sensor(0x03,0x00);
//		HI253_Sleep_Mode = (HI253_read_cmos_sensor(0x01) & 0xfe);
//		HI253_Sleep_Mode |= 0x00;
//		HI253_write_cmos_sensor(0x01, HI253_Sleep_Mode);

	}

	return 0;
}

int hy252_to_preview_320_240(void)
{
	int i, ret; 
	printk(KERN_INFO "[hy253]-to preview 320 * 240\n");	
	for(i = 0; i < (sizeof(hy252_to_preview_320_240_regs) / 2); i++)
	{
		ret = cam_hy252_write( (unsigned char *)(&hy252_to_preview_320_240_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	}


	return 0;
}

int hy252_to_preview_640_480(void)
{
	int i, ret; 
	printk(KERN_INFO "[hy253]-to preview 640 * 480\n");	
	for(i = 0; i < (sizeof(hy252_to_preview_640_480_regs) / 2); i++)
	{
		ret = cam_hy252_write( (unsigned char *)(&hy252_to_preview_640_480_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	}

	return 0;
}



/*
 *   set the color effect
 */

int hy252_sepia(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_sepia_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_sepia_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 
	return 0;
} 


int hy252_bluish(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_bluish_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_bluish_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 
	return 0;
} 


int hy252_greenish(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_greenish_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_greenish_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0;
}

int hy252_reddish(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_reddish_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_reddish_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0;
}


int hy252_yellowish(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_yellowish_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_yellowish_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0;
}


int hy252_bandw(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_bandw_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_bandw_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0;
}


int hy252_negative(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_negative_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_negative_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0;
}

int hy252_normal(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_normal_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_normal_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0;
}

/*
 * Light Mode
 */


int hy252_auto(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_auto_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_auto_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0;
}

int hy252_sunny(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_sunny_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_sunny_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0;
}

int hy252_cloudy(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_cloudy_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_cloudy_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0;
}

int hy252_office(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_office_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_office_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0;
}

int hy252_home(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_home_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_home_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0;
}


/*
 * Color saturation
 */

int hy252_saturation_0(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_saturation_0_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_saturation_0_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
} 

int hy252_saturation_1(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_saturation_1_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_saturation_1_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}

int hy252_saturation_2(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_saturation_2_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_saturation_2_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}

int hy252_saturation_3(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_saturation_3_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_saturation_3_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}

int hy252_saturation_4(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_saturation_4_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_saturation_4_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}


/*
 *  Brightness
 */

int hy252_brightness_0(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_brightness_0_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_brightness_0_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
} 

int hy252_brightness_1(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_brightness_1_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_brightness_1_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}

int hy252_brightness_2(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_brightness_2_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_brightness_2_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}

int hy252_brightness_3(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_brightness_3_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_brightness_3_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}

int hy252_brightness_4(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_brightness_4_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_brightness_4_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}

int hy252_brightness_5(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_brightness_5_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_brightness_5_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}

int hy252_brightness_6(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_brightness_6_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_brightness_6_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}


/*
 * Constrat
 */

int hy252_contrast_0(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_contrast_0_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_contrast_0_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
} 

int hy252_contrast_1(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_contrast_1_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_contrast_1_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}

int hy252_contrast_2(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_contrast_2_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_contrast_2_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}

int hy252_contrast_3(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_contrast_3_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_contrast_3_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}

int hy252_contrast_4(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_contrast_4_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_contrast_4_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}

int hy252_contrast_5(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_contrast_5_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_contrast_5_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}

int hy252_contrast_6(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_contrast_6_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_contrast_6_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}


/*
 * Sharpness
 */

int hy252_sharpness_0(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_sharpness_0_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_sharpness_0_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
} 

int hy252_sharpness_1(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_sharpness_1_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_sharpness_1_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}

int hy252_sharpness_2(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_sharpness_2_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_sharpness_2_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}

int hy252_sharpness_3(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_sharpness_3_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_sharpness_3_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}

int hy252_sharpness_4(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_sharpness_4_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_sharpness_4_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}

int hy252_sharpness_auto(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_sharpness_auto_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_sharpness_auto_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}


/*
 * Night Mode
 */

int hy252_night_mode_on(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_night_mode_on_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_night_mode_on_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}

int hy252_night_mode_off(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(hy252_night_mode_off_regs) / 2)); i++)
	{
		ret = cam_hy252_write( (unsigned char*)(&hy252_night_mode_off_regs[i]));
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	} 

	return 0 ;
}
int hy252_xx_set_mode(int value, struct imapx200_camif_param_t *param)
{
	int ret;
	printk( KERN_INFO "hy253 set mode %d \n",value);
	switch(value)
	{
		case INIT_SENSOR:
			ret = hy252_xx_reset(param);
			ret = hy252_start();
			break;

		case SWITCH_SENSOR_TO_HIGH_SVGA:
			ret = hy252_switch_high_svga();
			break;

		case SWITCH_SENSOR_TO_LOW_SVGA:
			ret = hy252_switch_low_svga();
			break;

		case SWITCH_SENSOR_TO_HIGH_XUGA:
			ret = hy252_svga_to_xuga(1600,1200);
			ret = hy252_switch_high_xuga();
			break;

		case SWITCH_SENSOR_TO_UPMID_XUGA:
			ret = hy252_svga_to_xuga(1280, 960);
			ret = hy252_switch_upmid_xuga();
			break;

		case SWITCH_SENSOR_TO_MID_XUGA:
			ret = hy252_svga_to_xuga(640,480);
			ret = hy252_switch_mid_xuga();
			break;

		case SWITCH_SENSOR_TO_LOW_XUGA:
			ret = hy252_svga_to_xuga(320,240);
			ret = hy252_switch_low_xuga();
			break;

		case SENSOR_TO_HIGH_PREVIEW:
			ret = hy252_start();
			ret = hy252_to_preview_640_480();
			break;

		case SENSOR_TO_LOW_PREVIEW:
			ret = hy252_start();
			ret = hy252_to_preview_320_240();
			break;
		case CLOSE_SENSOR:
			ret = hy252_close();
			break;
		default: 
			ret = -1;
			break;

	}
	return ret;

}

int hy252_xx_set_effect(int value)
{

	switch(value)                            
	{                                        
		case SENSOR_EFFECT_OFF:          
			hy252_normal();   
			break;                   
		case SENSOR_EFFECT_MONO:         
			hy252_bandw();    
			break;                   
		case SENSOR_EFFECT_NEGATIVE:     
			hy252_negative(); 
			break;                   
		case SENSOR_EFFECT_SOLARIZE:     
			hy252_yellowish();
			break;                   
		case SENSOR_EFFECT_PASTEL:       
			hy252_reddish();  
			break;                   
		case SENSOR_EFFECT_MOSAIC:       
			break;                   
		case SENSOR_EFFECT_RESIZE:       
			break;                   
		case SENSOR_EFFECT_SEPIA:        
			hy252_sepia();    
			break;                   
		case SENSOR_EFFECT_POSTERIZE:    
			hy252_bluish();   
			break;                   
		case SENSOR_EFFECT_WHITEBOARD:   
			break;                   
		case SENSOR_EFFECT_BLACKBOARD:   
			break;                   
		case SNESOR_EFFECT_AQUA:         
			hy252_greenish(); 
			break;                   
		default:                         
			break;                   
	}                                        

	return -1;
}

int hy252_xx_set_wb(int value)
{
	int ret;
	switch(value)
	{
		case SENSOR_WB_AUTO:
			hy252_auto();
			break;
		case SENSOR_WB_CUSTOM:
			break;
		case SENSOR_WB_INCANDESCENT:
			hy252_home();
			break;
		case SENSOR_WB_FLUORESCENT:
			hy252_office();
			break;
		case SENSOR_WB_DAYLIGHT:
			hy252_sunny();
			break;
		case SENSOR_WB_CLOUDY:
			hy252_cloudy();
			break;
		case SENSOR_WB_TWILIGHT:
			break;
		case SENSOR_WB_SHADE:
			break;
		default:
			break;
	}

	return -1;
}

int hy252_xx_set_antibanding(int value)
{
	return -1;
}

int hy252_xx_set_brightness(int value)
{
	return -1;
}

int hy252_xx_set_nightmode(int value)
{
	return -1;
}

#if CONFIG_SENSOR_TEST 
int hy252_xx_sensor_dbg(uint32_t value)
{
	unsigned char add = 0,val = 0,buf = 0;
	struct hy252_regval_list  regval;

	printk(KERN_ERR  "[IIC-DBUG]- Get %05x",value);
	switch(value & 0x00001){

		case 1:
			add = (value & 0xFF000) >> 12;
			val =  (value & 0x00FF0) >> 4;
			cam_hy252_read((unsigned char *)(&add), &buf, 1);
			printk(KERN_ERR "    [IIC_READ]- reg-%x,val-%x\n",add,buf);
			break;
		case 0:
			add = (value & 0xFF000) >> 12;
			val = (value & 0x00FF0) >> 4;
			regval.reg= add;
			regval.value = val;
			printk(KERN_ERR "    [IIC_WRTE]- reg-%x,val-%x\n",add,val);
			cam_hy252_write( (unsigned char *)(&regval));
			break;
		default:
			break;
	}
	return 0;
}
#endif

static uint32_t cmos_id_list[] = { 0x92, 0};
struct sensor_ops hy252_xx_ops = {
	.reset			= hy252_xx_reset,
	.set_mode		= hy252_xx_set_mode,
	.set_effect		= hy252_xx_set_effect,
	.set_wb			= hy252_xx_set_wb,
	.get_id                 = hy252_xx_get_id,
	.idlist                 = cmos_id_list,
	.addr                   = HY252_I2C_WADDR, 
	.name			= "XX:hy252",
	.hwid                   = 0,
	.pwdn                   = 1,
#if CONFIG_SENSOR_TEST
	.sensor_dbg		= hy252_xx_sensor_dbg,
#endif
	
};

static int __init hy252_xx_init(void)
{
	printk(KERN_INFO "hy252-ops-register");
	ops = &hy252_xx_ops;
	return imapx200_cam_sensor_register(ops);
}
static void __exit hy252_xx_exit(void)
{
	printk(KERN_INFO "hy252-ops-unregister");
	imapx200_cam_sensor_unregister(&hy252_xx_ops);
}

module_init(hy252_xx_init);
module_exit(hy252_xx_exit);









