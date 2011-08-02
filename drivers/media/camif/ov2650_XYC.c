/***************************************************************************** 
 ** ov_yinyucheng_2650.c 
 ** 
 ** Copyright (c) 2009~2014 ShangHai Infotm Ltd all rights reserved. 
 **     
 ** This program is free software; you can redistribute it and/or modify
 ** it under the terms of the GNU General Public License as published by
 ** the Free Software Foundation; either version 2 of the License, or
 ** (at your option) any later version.
 **             
 ** Description: sensor config for sensor of ov2650 production 
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

#include "ov2650_XYC.h"

static int cam_ov2650_read(unsigned char IICAddr, unsigned char * ByteAddr, unsigned char *Data, unsigned int Size)
{                                                                                                           
	struct i2c_adapter *adapter;                                                                        
	int ret;                                                                                            

	struct i2c_msg msgs[] = {                                                                           
		{                                                                                           
			.addr   = IICAddr,                                                                  
			.flags  = 0,                                                                        
			.len            = 2,                                                                
			.buf            = ByteAddr,                                                         

		},{                                                                                         
			.addr   = IICAddr,                                                                  
			.flags  = I2C_M_RD,                                                                 
			.len            = Size,                                                             
			.buf            = Data,                                                             
		}                                                                                           
	};                                                                                                  
	adapter = i2c_get_adapter(2);                                                                       
	if (!adapter)                                                                                       
	{                                                                                                   
		return -1;                                                                                  
	}                                                                                                   
	ret = i2c_transfer(adapter, msgs, 2);                                                               
	if ( ret != 2 )                                                                                     
	{                                                                                                   
		return -1;                                                                                  
	}                                                                                                   

	return 0;                                                                                           
}                                                                                                           


static int cam_ov2650_write(unsigned char IICAddr, unsigned char * data, enum senr_type  sensor) 
{                                                                                           
	struct i2c_adapter *adapter;                                                        
	unsigned int    size = 3;                                                           
	unsigned int ret;                                                                   

	struct i2c_msg msgs[] = {                                                           
		{                                                                           
			.addr   = IICAddr,                                                  
			.flags  = 0,                                                        
			.len            = size,                                             
			.buf            = data,                                             
		}                                                                           
	};                                                                                  

	adapter = i2c_get_adapter(2);                                                       
	if (!adapter)                                                                       
	{                                                                                   
		return -1;                                                                  
	}                                                                                   
	ret = i2c_transfer(adapter, msgs, 1) ;                                              
	if (ret!= 1)                                                                        
	{                                                                                   
		return -1;                                                                  
	}                                                                                   

	return 0;                                                                           
}                                                                                           

int ov2650_xyc_power_on(void)
{
    /*
	 * camera interface power supply enable
	 *
	 */
	uint32_t pow_addr, pwdn_addr;

	pow_addr = __imapx_name_to_gpio(CONFIG_IG_CAMIF0_SUPPLY);
	if(pow_addr == IMAPX_GPIO_ERROR) {                       
		printk(KERN_ERR "failed to get spken pin.\n");   
		return -1;                                       
	}                                                        
	imapx_gpio_setpin(pow_addr, 1, IG_NORMAL);               

	/*
	 *    camera inerface pwdn enable
	 *
	 */
	pwdn_addr = __imapx_name_to_gpio(CONFIG_IG_CAMIF0_PND);
	if(pow_addr == IMAPX_GPIO_ERROR) {                       
		printk(KERN_ERR "failed to get spken pin.\n");   
		return -1;                                       
	}                                                        
	imapx_gpio_setpin(pwdn_addr, 0, IG_NORMAL);               

	return 0;

}

int ov2650_xyc_power_off(void)
{

    uint32_t pow_addr, pwdn_addr;
	/*
	 * disable power supply
	 */
	pow_addr = __imapx_name_to_gpio(CONFIG_IG_CAMIF0_SUPPLY);
	if(pow_addr == IMAPX_GPIO_ERROR) {
		        printk(KERN_ERR "failed to get spken pin.\n");
			        return -1;
	}
	imapx_gpio_setpin(pow_addr, 0, IG_NORMAL);

	/*
	 *  set sensor to sleep mode
	 */

	pwdn_addr = __imapx_name_to_gpio(CONFIG_IG_CAMIF0_PND);
	if(pow_addr == IMAPX_GPIO_ERROR) {                       
		printk(KERN_ERR "failed to get spken pin.\n");   
		return -1;                                       
	}                                                        
	imapx_gpio_setpin(pwdn_addr, 1, IG_NORMAL);               

	return 0;
}




int ov2650_xyc_reset(struct imapx200_camif_param_t *param)
{
	uint32_t tmp = 0;

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
	tmp = readl(param->ioaddr+IMAP_CIGCTRL);

	return 0;
}
/*
 *   init & close sensor 
 */

int ov2650_xyc_start(void){
	char buf = 0;
	int i, ret; 

	printk(KERN_INFO "ov2650_xyc_start\n");	
	cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(&ov2650_init_regs[0]), OV2655_XYC);
	msleep(50);

	cam_ov2650_read(OV2650_I2C_RADDR, (unsigned char *)(&OV2650_PIDH), &buf, 1);
	if(buf != 0x26)
		printk("i2c config OV2650 wrong with %d\n",buf); 
	cam_ov2650_read(OV2650_I2C_RADDR, (unsigned char *)(&OV2650_PIDL), &buf, 1);
	if(buf != 0x56)
		printk("i2c config OV2650 wrong with %d\n",buf);

	for(i = 0; i < ((sizeof(ov2650_init_regs) / 3) -1); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(&ov2650_init_regs[i+1]), OV2655_XYC);
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

int ov2650_xyc_close(void)
{
	int i, ret;
	uint32_t tmp;
	printk(KERN_INFO "ov2650_xyc_close\n");	

	for(i = 0; i < ((sizeof(ov2650_stop_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(&ov2650_stop_regs[i]), OV2655_XYC);
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
int  ov2650_xyc_switch_low_svga(void)
{
	int i, ret;	

	printk(KERN_INFO "ov2650_xyc_switch_low_svga\n");	
	for(i = 0; i < ((sizeof(ov2650_svga_low_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(&ov2650_svga_low_regs[i]), OV2655_XYC);
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
 	 * used to change fps to 18
	 */
int  ov2650_xyc_switch_high_svga(void)
{
	int i, ret;

	printk(KERN_INFO "ov2650_xyc_switch_high_svga\n");	
	for(i = 0; i < ((sizeof(ov2650_svga_high_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(&ov2650_svga_high_regs[i]), OV2655_XYC);
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
  * 1600x1200
  */
int  ov2650_xyc_switch_high_xuga(void)
{
	int i, ret;

	cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(ov2650_before), OV2655_XYC);
	mdelay(5);
	for(i = 0; i < ((sizeof(ov2650_xuga_high_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(&ov2650_xuga_high_regs[i]), OV2655_XYC);
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	}
	cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(ov2650_after), OV2655_XYC);
	return 0;
}

/*
 * 1280x960
 */
int ov2650_xyc_switch_upmid_xuga(void)
{
	int i, ret;

	cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(ov2650_before), OV2655_XYC);
	mdelay(5);
	for(i = 0; i < ((sizeof(ov2650_xuga_upmid_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(&ov2650_xuga_upmid_regs[i]), OV2655_XYC);
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	}
	cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(ov2650_after), OV2655_XYC);
	return 0;
}

/*
 * 800x600
 */
int  ov2650_xyc_switch_mid_xuga(void)
{
	int i, ret;

	cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(ov2650_before), OV2655_XYC);
	mdelay(5);
	for(i = 0; i < ((sizeof(ov2650_xuga_mid_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(&ov2650_xuga_mid_regs[i]), OV2655_XYC);
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	}
	cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(ov2650_after), OV2655_XYC);
	return 0;
}
/*
 * 640x480
 */

int  ov2650_xyc_switch_low_xuga(void)
{
	int i, ret;

	cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(ov2650_before), OV2655_XYC);
	mdelay(5);
	for(i = 0; i < ((sizeof(ov2650_xuga_low_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(&ov2650_xuga_low_regs[i]), OV2655_XYC);
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	}
	cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(ov2650_after), OV2655_XYC);
	return 0;
}

/*
 * svga --> xuga
 */

int ov2650_xyc_svga_to_xuga(void){
	int i, ret;
	char gain = 0;

	char reg0x3013;
	char reg0x3002, reg0x3003;
	char reg0x302d, reg0x302e;
	char reg0x3000;
#ifdef D50HZ
	char reg0x3070, reg0x3071;
#else
	char reg0x3072, reg0x3073;
#endif

	uint32_t shutter;
	uint32_t extra_lines;
	uint32_t preview_exposure;
	uint32_t preview_gain16;
	uint32_t preview_dummy_pixel;
	uint32_t capture_dummy_pixel;
	uint32_t capture_dummy_line;
	uint32_t preview_pclk_frequency;
	uint32_t capture_pclk_frequency;
	uint32_t capture_max_gain;
	uint32_t capture_max_gain16;
	uint32_t preview_line_width;
	uint32_t capture_line_width;
	uint32_t capture_maximum_shutter;
	uint32_t capture_exposure;
	uint32_t preview_banding_filter;
	uint32_t capture_banding_filter = 0; 
	uint32_t gain_exposure;
	uint32_t capture_gain16;
	uint32_t capture_gain;

	struct ov2650_regval_list ov2650_stop_preview[] = {
		{0x30, 0x13, 0x00},
	};
	struct ov2650_regval_list ov2650_3002[] = {
		{0x30, 0x02, 0x00},
	};
	struct ov2650_regval_list ov2650_3003[] = {
		{0x30, 0x03, 0x00},
	};
	struct ov2650_regval_list ov2650_302d[] = {
		{0x30, 0x2d, 0x00},
	};
	struct ov2650_regval_list ov2650_302e[] = {
		{0x30, 0x2e, 0x00},
	};
	struct ov2650_regval_list ov2650_3000[] = {
		{0x30, 0x00, 0x00},
	};

	cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(ov2650_before), OV2655_XYC);
	mdelay(5);

	cam_ov2650_read(OV2650_I2C_RADDR, (unsigned char *)(&OV2650_3013), &reg0x3013, 1);
	reg0x3013 = reg0x3013 & 0xfa;
	ov2650_stop_preview->value	= reg0x3013;	

	ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(ov2650_stop_preview), OV2655_XYC);
	if(ret)
	{
		camif_error("Failed to transfer data to i2c\n");
		return -1;
	}
	else{
	}

	cam_ov2650_read(OV2650_I2C_RADDR, (unsigned char *)(&OV2650_3002), &reg0x3002, 1);
	cam_ov2650_read(OV2650_I2C_RADDR, (unsigned char *)(&OV2650_3003), &reg0x3003, 1);
	shutter = (((uint32_t)reg0x3002)<<8) + (uint32_t)reg0x3003;

	preview_exposure = shutter;

	cam_ov2650_read(OV2650_I2C_RADDR, (unsigned char *)(&OV2650_3000), &reg0x3000, 1);
	preview_gain16 = (uint32_t)(16 + (reg0x3000 & 0x0f));
	preview_gain16 = (preview_gain16 * (((reg0x3000 & 0x80)>>7)+1) * (((reg0x3000 & 0x40)>>6)+1) * (((reg0x3000 & 0x20)>>5)+1) * (((reg0x3000 & 0x10)>>4)+1));

	preview_dummy_pixel = 0;
	capture_dummy_pixel = 0;
	capture_dummy_line = 0;
	preview_pclk_frequency = 540;  	
	capture_pclk_frequency = 324;	

	capture_max_gain   = 1;		
	capture_max_gain16 = capture_max_gain * 16;
	preview_line_width = 1940 + preview_dummy_pixel;
	capture_line_width = 1940 + capture_dummy_pixel;
	capture_maximum_shutter = 1236 + capture_dummy_line;
	capture_exposure = preview_exposure * capture_pclk_frequency * preview_line_width;
	capture_exposure = capture_exposure / (preview_pclk_frequency * capture_line_width);

#ifdef D50HZ
	cam_ov2650_read(OV2650_I2C_RADDR, (unsigned char *)(&OV2650_3070), &reg0x3070, 1);
	cam_ov2650_read(OV2650_I2C_RADDR, (unsigned char *)(&OV2650_3071), &reg0x3071, 1);
	preview_banding_filter = (((uint32_t)reg0x3071)<<8) + (uint32_t)reg0x3070;
#else
	cam_ov2650_read(OV2650_I2C_RADDR, (unsigned char *)(&OV2650_3072), &reg0x3072, 1);
	cam_ov2650_read(OV2650_I2C_RADDR, (unsigned char *)(&OV2650_3073), &reg0x3073, 1);
	preview_banding_filter = (((uint32_t)reg0x3073)<<8) + (uint32_t)reg0x3072;
#endif

#if 0
	capture_banding_filter = preview_banding_filter * capture_pclk_frequency;
	capture_banding_filter = capture_banding_filter * preview_line_width;
	capture_banding_filter = capture_banding_filter / preview_pclk_frequency;
	capture_banding_filter = capture_banding_filter / capture_line_width;
#endif
#ifdef D50HZ
	capture_banding_filter = 84;
#else
	capture_banding_filter = 70;
#endif

	gain_exposure = preview_gain16 * capture_exposure;
	if( gain_exposure < capture_banding_filter * 16 ) {
		capture_exposure = gain_exposure / 16;
		capture_gain16 = (gain_exposure*2 + 1)/capture_exposure;
		capture_gain16 = capture_gain16 >> 1;
	}
	else {
		if( gain_exposure > capture_maximum_shutter * 16 ) {
			capture_exposure = capture_maximum_shutter;
			capture_gain16 = (gain_exposure*2 + 1)/capture_maximum_shutter;
			capture_gain16 = capture_gain16 >> 1;
			if( capture_gain16 > capture_max_gain16 ) {
#if 0
				capture_exposure = (gain_exposure + (gain_exposure/10)) * capture_banding_filter;
				capture_exposure = capture_exposure / (16 * capture_max_gain16 * capture_banding_filter);	
#endif
#if 1
				capture_exposure = (gain_exposure + (gain_exposure/10)) / preview_gain16;
				capture_exposure = capture_exposure - (capture_exposure % capture_banding_filter);
#endif
				capture_gain16 = (gain_exposure*2 + 1)/capture_exposure;
				capture_gain16 = capture_gain16 >> 1;
			}
			else {
#if 0
				capture_exposure = capture_exposure * capture_banding_filter;
				capture_exposure = capture_exposure / capture_banding_filter;
#endif
#if 1
				capture_exposure = capture_exposure - (capture_exposure % capture_banding_filter);
#endif
				capture_gain16 = (gain_exposure*2 + 1)/capture_maximum_shutter;
				capture_gain16 = capture_gain16 >> 1;
			}
		}
		else {
#if 0
			capture_exposure = gain_exposure * capture_banding_filter;
			capture_exposure = capture_exposure / (16 * capture_banding_filter);
#endif
#if 1
			capture_exposure = gain_exposure / 16;
			capture_exposure = capture_exposure - (capture_exposure % capture_banding_filter);
#endif
			capture_gain16 = (gain_exposure*2 + 1)/capture_exposure;
			capture_gain16 = capture_gain16 >> 1;
		}
	}

	for(i = 0; i < ((sizeof(ov2650_xuga_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(&ov2650_xuga_regs[i]), OV2655_XYC);
		if(ret)
		{
			camif_error("Failed to transfer data to i2c\n");
			return -1;
		}
		else{
		}
	}

	if( capture_exposure > capture_maximum_shutter ) {
		shutter = capture_maximum_shutter;
		extra_lines = capture_exposure - capture_maximum_shutter;
	}
	else {
		shutter = capture_exposure;
		extra_lines = 0;
	}
	reg0x3003 = shutter & 0x00ff;
	reg0x3002 = (shutter >>8) & 0x00ff;
	ov2650_3002->value = reg0x3002;
	ov2650_3003->value = reg0x3003;
	ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(ov2650_3002), OV2655_XYC);
	ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(ov2650_3003), OV2655_XYC);
	if(ret)
	{
		camif_error("Failed to transfer data to i2c\n");
		return -1;
	}
	else{
	}

	reg0x302e = extra_lines & 0x00ff;
	reg0x302d = extra_lines >> 8;
	ov2650_302d->value = reg0x302d;
	ov2650_302e->value = reg0x302e;
	ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(ov2650_302d), OV2655_XYC);
	ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(ov2650_302e), OV2655_XYC);
	if(ret)
	{
		camif_error("Failed to transfer data to i2c\n");
		return -1;
	}
	else{
	}


	capture_gain = capture_gain16;
	if( capture_gain16 > 16 ) {
		capture_gain16 = capture_gain16 / 2;
		gain = 0x10;
	}
	if( capture_gain16 > 16 ) {
		capture_gain16 = capture_gain16 / 2;
		gain = gain|0x20;
	}
	if( capture_gain16 > 16 ) {
		capture_gain16 = capture_gain16 / 2;
		gain = gain|0x40;
	}
	if( capture_gain16 > 16 ) {
		capture_gain16 = capture_gain16 / 2;
		gain = gain|0x80;
	}
	gain = gain | (char)(capture_gain - 16);
	ov2650_3000->value = gain;
	ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(ov2650_3000), OV2655_XYC);
	if(ret)
	{
		camif_error("Failed to transfer data to i2c\n");
		return -1;
	}
	else{
	}

	mdelay(30);
	cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(ov2650_after), OV2655_XYC);

	return 0;
}

int ov2650_xyc_to_preview_320_240(void)
{
	int i, ret;
	char reg0x3013;
	struct ov2650_regval_list ov2650_3013[] = {
		{0x30, 0x13, 0x00},
	};

	cam_ov2650_read(OV2650_I2C_RADDR, (unsigned char *)(&OV2650_3013), &reg0x3013,1);
	reg0x3013 = reg0x3013 | 0x05;   
	ov2650_3013->value = reg0x3013;
	ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(ov2650_3013), OV2655_XYC);                       

	for(i = 0; i < ((sizeof(ov2650_to_preview_320_240_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(&ov2650_to_preview_320_240_regs[i]), OV2655_XYC);
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

int ov2650_xyc_to_preview_640_480(void)
{
	int i, ret;
	char reg0x3013;
	struct ov2650_regval_list ov2650_3013[] = {
		{0x30, 0x13, 0x00},
	};

	cam_ov2650_read(OV2650_I2C_RADDR, (unsigned char *)(&OV2650_3013), &reg0x3013,1);
	reg0x3013 = reg0x3013 | 0x05;   
	ov2650_3013->value = reg0x3013;
	ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(ov2650_3013), OV2655_XYC);                       

	for(i = 0; i < ((sizeof(ov2650_to_preview_640_480_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char *)(&ov2650_to_preview_640_480_regs[i]), OV2655_XYC);
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

int ov2650_xyc_sepia(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_sepia_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_sepia_regs[i]), OV2655_XYC);
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


int ov2650_xyc_bluish(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_bluish_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_bluish_regs[i]), OV2655_XYC);
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


int ov2650_xyc_greenish(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_greenish_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_greenish_regs[i]), OV2655_XYC);
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

int ov2650_xyc_reddish(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_reddish_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_reddish_regs[i]), OV2655_XYC);
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


int ov2650_xyc_yellowish(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_yellowish_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_yellowish_regs[i]), OV2655_XYC);
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


int ov2650_xyc_bandw(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_bandw_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_bandw_regs[i]), OV2655_XYC);
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


int ov2650_xyc_negative(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_negative_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_negative_regs[i]), OV2655_XYC);
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

int ov2650_xyc_normal(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_normal_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_normal_regs[i]), OV2655_XYC);
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


int ov2650_xyc_auto(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_auto_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_auto_regs[i]), OV2655_XYC);
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

int ov2650_xyc_sunny(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_sunny_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_sunny_regs[i]), OV2655_XYC);
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

int ov2650_xyc_cloudy(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_cloudy_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_cloudy_regs[i]), OV2655_XYC);
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

int ov2650_xyc_office(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_office_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_office_regs[i]), OV2655_XYC);
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

int ov2650_xyc_home(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_home_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_home_regs[i]), OV2655_XYC);
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

int ov2650_xyc_saturation_0(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_saturation_0_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_saturation_0_regs[i]), OV2655_XYC);
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

int ov2650_xyc_saturation_1(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_saturation_1_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_saturation_1_regs[i]), OV2655_XYC);
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

int ov2650_xyc_saturation_2(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_saturation_2_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_saturation_2_regs[i]), OV2655_XYC);
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

int ov2650_xyc_saturation_3(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_saturation_3_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_saturation_3_regs[i]), OV2655_XYC);
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

int ov2650_xyc_saturation_4(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_saturation_4_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_saturation_4_regs[i]), OV2655_XYC);
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

int ov2650_xyc_brightness_0(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_brightness_0_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_brightness_0_regs[i]), OV2655_XYC);
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

int ov2650_xyc_brightness_1(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_brightness_1_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_brightness_1_regs[i]), OV2655_XYC);
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

int ov2650_xyc_brightness_2(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_brightness_2_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_brightness_2_regs[i]), OV2655_XYC);
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

int ov2650_xyc_brightness_3(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_brightness_3_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_brightness_3_regs[i]), OV2655_XYC);
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

int ov2650_xyc_brightness_4(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_brightness_4_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_brightness_4_regs[i]), OV2655_XYC);
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

int ov2650_xyc_brightness_5(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_brightness_5_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_brightness_5_regs[i]), OV2655_XYC);
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

int ov2650_xyc_brightness_6(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_brightness_6_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_brightness_6_regs[i]), OV2655_XYC);
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

int ov2650_xyc_contrast_0(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_contrast_0_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_contrast_0_regs[i]), OV2655_XYC);
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

int ov2650_xyc_contrast_1(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_contrast_1_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_contrast_1_regs[i]), OV2655_XYC);
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

int ov2650_xyc_contrast_2(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_contrast_2_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_contrast_2_regs[i]), OV2655_XYC);
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

int ov2650_xyc_contrast_3(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_contrast_3_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_contrast_3_regs[i]), OV2655_XYC);
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

int ov2650_xyc_contrast_4(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_contrast_4_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_contrast_4_regs[i]), OV2655_XYC);
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

int ov2650_xyc_contrast_5(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_contrast_5_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_contrast_5_regs[i]), OV2655_XYC);
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

int ov2650_xyc_contrast_6(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_contrast_6_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_contrast_6_regs[i]), OV2655_XYC);
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

int ov2650_xyc_sharpness_0(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_sharpness_0_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_sharpness_0_regs[i]), OV2655_XYC);
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

int ov2650_xyc_sharpness_1(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_sharpness_1_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_sharpness_1_regs[i]), OV2655_XYC);
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

int ov2650_xyc_sharpness_2(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_sharpness_2_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_sharpness_2_regs[i]), OV2655_XYC);
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

int ov2650_xyc_sharpness_3(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_sharpness_3_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_sharpness_3_regs[i]), OV2655_XYC);
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

int ov2650_xyc_sharpness_4(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_sharpness_4_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_sharpness_4_regs[i]), OV2655_XYC);
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

int ov2650_xyc_sharpness_auto(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_sharpness_auto_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_sharpness_auto_regs[i]), OV2655_XYC);
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

int ov2650_xyc_night_mode_on(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_night_mode_on_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_night_mode_on_regs[i]), OV2655_XYC);
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

int ov2650_xyc_night_mode_off(void)
{ 
	int i, ret; 

	for(i = 0; i< ((sizeof(ov2650_night_mode_off_regs) / 3)); i++)
	{
		ret = cam_ov2650_write(OV2650_I2C_WADDR, (unsigned char*)(&ov2650_night_mode_off_regs[i]), OV2655_XYC);
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


int ov2650_xyc_set_mode(int value, struct imapx200_camif_param_t *param)
{
	int ret;
	printk( KERN_INFO "ov2650 set mode %d \n",value);
	switch(value)
	{
		case INIT_SENSOR:
			ret = ov2650_xyc_start();
			break;

		case SWITCH_SENSOR_TO_HIGH_SVGA:
			ret = ov2650_xyc_switch_high_svga();
			break;

		case SWITCH_SENSOR_TO_LOW_SVGA:
			ret = ov2650_xyc_switch_low_svga();
			break;

		case SWITCH_SENSOR_TO_HIGH_XUGA:
			ret = ov2650_xyc_svga_to_xuga();
			ret = ov2650_xyc_switch_high_xuga();
			break;

		case SWITCH_SENSOR_TO_UPMID_XUGA:
			ret = ov2650_xyc_svga_to_xuga();
			ret = ov2650_xyc_switch_upmid_xuga();
			break;

		case SWITCH_SENSOR_TO_MID_XUGA:
			ret = ov2650_xyc_svga_to_xuga();
			ret = ov2650_xyc_switch_mid_xuga();
			break;

		case SWITCH_SENSOR_TO_LOW_XUGA:
			ret = ov2650_xyc_switch_low_xuga();
			break;

		case SENSOR_TO_HIGH_PREVIEW:
			ret = ov2650_xyc_to_preview_640_480();
			break;

		case SENSOR_TO_LOW_PREVIEW:
			ret = ov2650_xyc_to_preview_320_240();
			break;
		case CLOSE_SENSOR:
			ret = ov2650_xyc_close();
			break;
		default: 
			ret = -1;
			break;

	}
	return ret;

}

int ov2650_xyc_set_effect(int value)
{

	switch(value)                            
	{                                        
		case SENSOR_EFFECT_OFF:          
			ov2650_xyc_normal();   
			break;                   
		case SENSOR_EFFECT_MONO:         
			ov2650_xyc_bandw();    
			break;                   
		case SENSOR_EFFECT_NEGATIVE:     
			ov2650_xyc_negative(); 
			break;                   
		case SENSOR_EFFECT_SOLARIZE:     
			ov2650_xyc_yellowish();
			break;                   
		case SENSOR_EFFECT_PASTEL:       
			ov2650_xyc_reddish();  
			break;                   
		case SENSOR_EFFECT_MOSAIC:       
			break;                   
		case SENSOR_EFFECT_RESIZE:       
			break;                   
		case SENSOR_EFFECT_SEPIA:        
			ov2650_xyc_sepia();    
			break;                   
		case SENSOR_EFFECT_POSTERIZE:    
			ov2650_xyc_bluish();   
			break;                   
		case SENSOR_EFFECT_WHITEBOARD:   
			break;                   
		case SENSOR_EFFECT_BLACKBOARD:   
			break;                   
		case SNESOR_EFFECT_AQUA:         
			ov2650_xyc_greenish(); 
			break;                   
		default:                         
			break;                   
	}                                        

	return -1;
}

int ov2650_xyc_set_wb(int value)
{

	switch(value)
	{
		case SENSOR_WB_AUTO:
			ov2650_xyc_auto();
			break;
		case SENSOR_WB_CUSTOM:
			break;
		case SENSOR_WB_INCANDESCENT:
			ov2650_xyc_home();
			break;
		case SENSOR_WB_FLUORESCENT:
			ov2650_xyc_office();
			break;
		case SENSOR_WB_DAYLIGHT:
			ov2650_xyc_sunny();
			break;
		case SENSOR_WB_CLOUDY:
			ov2650_xyc_cloudy();
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

int ov2650_xyc_set_antibanding(int value)
{
	return -1;
}

int ov2650_xyc_set_brightness(int value)
{
	return -1;
}

int ov2650_xyc_set_nightmode(int value)
{
	return -1;
}


struct sensor_ops ov2650_xyc_ops = {
	.power_on   		= ov2650_xyc_power_on,
	.power_off		= ov2650_xyc_power_off,
	.reset			= ov2650_xyc_reset,
	.set_mode		= ov2650_xyc_set_mode,
	.set_effect		= ov2650_xyc_set_effect,
	.set_wb			= ov2650_xyc_set_wb,
};

EXPORT_SYMBOL(ov2650_xyc_ops);








