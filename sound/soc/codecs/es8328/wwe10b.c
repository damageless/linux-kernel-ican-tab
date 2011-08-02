/*
 * es8328.c -- es8328 ALSA SoC audio driver
 *
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Richard Purdie <richard@openedhand.com>
 *
 * Based on es8328.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
//#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#ifdef CONFIG_HHBF_FAST_REBOOT
#include <asm/reboot.h>
#endif

#include "es8328.h"

#define AUDIO_NAME "es8328"
#define es8328_VERSION "v0.12"
#include <mach/imapx_gpio.h>
#include <asm/io.h>

#ifdef CONFIG_FAKE_PM
#include <plat/fake_pm.h>
#endif
unsigned int system_mute;
unsigned int system_mute_state;
static mute_initial = 0;
/*
 * Debug
 */

//#define es8328_DEBUG 1

#ifdef es8328_DEBUG
#define dbg(format, arg...) \
	printk(KERN_DEBUG AUDIO_NAME ": " format "\n" , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif
#define err(format, arg...) \
	printk(KERN_ERR AUDIO_NAME ": " format "\n" , ## arg)
#define info(format, arg...) \
	printk(KERN_INFO AUDIO_NAME ": " format "\n" , ## arg)
#define warn(format, arg...) \
	printk(KERN_WARNING AUDIO_NAME ": " format "\n" , ## arg)

/* codec private data */
#ifndef	CONFIG_HHTECH_MINIPMP
struct es8328_priv {
	unsigned int sysclk;
};
#else
static unsigned es8328_sysclk;

void (*hhbf_audio_switch)(int flag) = NULL;
EXPORT_SYMBOL(hhbf_audio_switch);
static unsigned short init_reboot = 0;
#endif//CONFIG_HHTECH_MINIPMP

extern void imap_iokey_spken(int);
/*
 * es8328 register cache
 * We can't read the es8328 register space when we
 * are using 2 wire for device control, so we cache them instead.
 */
static const u16 es8328_reg[] = {
	0x00b7, 0x0097, 0x0000, 0x0000,  /*  0 */
	0x0000, 0x0008, 0x0000, 0x002a,  /*  4 */
	0x0000, 0x0000, 0x007F, 0x007F,  /*  8 */
	0x000f, 0x000f, 0x0000, 0x0000,  /* 12 */
	0x0080, 0x007b, 0x0000, 0x0032,  /* 16 */
	0x0000, 0x00E0, 0x00E0, 0x00c0,  /* 20 */
	0x0000, 0x000e, 0x0000, 0x0000,  /* 24 */
	0x0000, 0x0000, 0x0000, 0x0000,  /* 28 */
	0x0000, 0x0000, 0x0050, 0x0050,  /* 32 */
	0x0050, 0x0050, 0x0050, 0x0050,  /* 36 */
	0x0000, 0x0000, 0x0079,          /* 40 */
};

/*
 * read es8328 register cache
 */
static inline unsigned int es8328_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;
#ifndef	CONFIG_HHTECH_MINIPMP
	if (reg > es8328_CACHE_REGNUM)
#else// mhfan
	if (reg > ARRAY_SIZE(es8328_reg))
#endif//CONFIG_HHTECH_MINIPMP
		return -1;
	return cache[reg];
}

/*
 * write es8328 register cache
 */
static inline void es8328_write_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
#ifndef	CONFIG_HHTECH_MINIPMP
	if (reg > es8328_CACHE_REGNUM)
#else// mhfan
	if (reg > ARRAY_SIZE(es8328_reg))
#endif//CONFIG_HHTECH_MINIPMP
		return;
	cache[reg] = value;
}

#if 0//def	CONFIG_HHTECH_MINIPMP
static int playtvo = 0;
#endif//CONFIG_HHTECH_MINIPMP

static int es8328_read(struct snd_soc_codec *codec, unsigned int reg)
{
	u8 data[1];

	data[0] = reg ;
	codec->hw_write(codec->control_data, data, 1);
//	printk("************data0 is %d\n", data[0]);
//	msleep(5);
	data[0]= 0;
	i2c_master_recv(codec->control_data, data,1);
//	printk("************data1 is %d\n", data[0]);
}


static int es8328_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	u8 data[2];
	/* data is
	 *   D15..D9 es8328 register offset
	 *   D8...D0 register data
	 */
	data[0] = reg ;
	data[1] = value;
	if (codec->hw_write(codec->control_data, data, 2) == 2)
		return 0;
	else
	{
		printk("i2c_transfer_error!\n");
		return -EIO;
	}
//return 0;
}


static const struct snd_kcontrol_new es8328_snd_controls[] = {

};

/* add non dapm controls */
static int es8328_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(es8328_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				snd_soc_cnew(&es8328_snd_controls[i],codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}


static const struct snd_soc_dapm_widget es8328_dapm_widgets[] = {

};

static const char *audio_map[][3] = {

	{NULL, NULL, NULL},
};
static int es8328_add_widgets(struct snd_soc_codec *codec)
{
	int i;

	for(i = 0; i < ARRAY_SIZE(es8328_dapm_widgets); i++) {
		snd_soc_dapm_new_control(codec, &es8328_dapm_widgets[i]);
	}
#if 1
	/* set up audio path audio_mapnects */
	for(i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(codec, audio_map[i][0],
			audio_map[i][1], audio_map[i][2]);
	}
#endif

//	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));
	snd_soc_dapm_new_widgets(codec);
	return 0;
}

struct _coeff_div {
	u32 mclk;
	u32 rate;
	u16 fs;
	u8 sr:5;
	u8 usb:1;
};

/* codec hifi mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {
	/* 8k */
	{12288000, 8000, 1536, 0x6, 0x0},
	{11289600, 8000, 1408, 0x16, 0x0},
	{18432000, 8000, 2304, 0x7, 0x0},
	{16934400, 8000, 2112, 0x17, 0x0},
	{12000000, 8000, 1500, 0x6, 0x1},

	/* 11.025k */
	{11289600, 11025, 1024, 0x18, 0x0},
	{16934400, 11025, 1536, 0x19, 0x0},
	{12000000, 11025, 1088, 0x19, 0x1},

	/* 16k */
	{12288000, 16000, 768, 0xa, 0x0},
	{18432000, 16000, 1152, 0xb, 0x0},
	{12000000, 16000, 750, 0xa, 0x1},

	/* 22.05k */
	{11289600, 22050, 512, 0x1a, 0x0},
	{16934400, 22050, 768, 0x1b, 0x0},
	{12000000, 22050, 544, 0x1b, 0x1},

	/* 32k */
	{12288000, 32000, 384, 0xc, 0x0},
	{18432000, 32000, 576, 0xd, 0x0},
	{12000000, 32000, 375, 0xc, 0x1},	// mhfan

	/* 44.1k */
	{11289600, 44100, 256, 0x10, 0x0},
	{16934400, 44100, 384, 0x11, 0x0},
	{12000000, 44100, 272, 0x11, 0x1},

	/* 48k */
	{12288000, 48000, 256, 0x0, 0x0},
	{18432000, 48000, 384, 0x1, 0x0},
	{12000000, 48000, 250, 0x0, 0x1},

	/* 88.2k */
	{11289600, 88200, 128, 0x1e, 0x0},
	{16934400, 88200, 192, 0x1f, 0x0},
	{12000000, 88200, 136, 0x1f, 0x1},

	/* 96k */
	{12288000, 96000, 128, 0xe, 0x0},
	{18432000, 96000, 192, 0xf, 0x0},
	{12000000, 96000, 125, 0xe, 0x1},
};

static inline int get_coeff(int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk)
			return i;
	}

	printk(KERN_ERR "es8328: could not get coeff for mclk %d @ rate %d\n",
		mclk, rate);
	return -EINVAL;
}

#if 1 
static int es8328_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
		int div_id, int div)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;
	return 0;
}

#endif
static int es8328_set_dai_sysclk(struct snd_soc_codec_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
#ifndef	CONFIG_HHTECH_MINIPMP
	struct snd_soc_codec *codec = codec_dai->codec;
	struct es8328_priv *es8328 = codec->private_data;
#endif//CONFIG_HHTECH_MINIPMP
//	printk("freq is %ld \n",freq);

	switch (freq) {
	case 11289600:
	case 12000000:
	case 12288000:
	case 16934400:
	case 18432000:
#ifndef	CONFIG_HHTECH_MINIPMP
		es8328->sysclk = freq;
#else// mhfan
	case 24576000:
		es8328_sysclk = freq;
#endif//CONFIG_HHTECH_MINIPMP
		return 0;
	}
	return -EINVAL;
}

static int es8328_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = 0;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface = 0x0040;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x0002;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x0001;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= 0x0003;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface |= 0x0013;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x0090;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x0080;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x0010;
		break;
	default:
		return -EINVAL;
	}

#if 1 
#ifdef	CONFIG_HHTECH_MINIPMP
	iface |= 0x20;	// XXX: mhfan
#endif//CONFIG_HHTECH_MINIPMP
#endif //lzcx
	/*
	if(iface != es8328_read_reg_cache(codec, es8328_IFACE))
		es8328_write(codec, es8328_IFACE, iface);
		*/
	return 0;
}

static int es8328_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	static int ifirst = 1;

#ifdef CONFIG_FAKE_PM
	unsigned int temp;

	if(if_in_suspend == 1)
	{
		es8328_write(codec, 0x19,  0x36);//0x0d
	}
	else 
	{
		es8328_write(codec, 0x19,  0x32);//0x0d
#endif
		if (params_rate(params)== 8000)
		{
			//printk("sound record!\n");
			es8328_write(codec, ES8328_CHIPPOWER,  0x55);//0x0d
		}
		if (params_rate(params)== 44100)
		{
			//printk("sound track!\n");
		//	es8328_write(codec, ES8328_CHIPPOWER,  0xaa);//0x0d
		}
#ifdef CONFIG_FAKE_PM
	}
#endif

	return 0;
}

static int es8328_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;

	if (mute_initial == 0){
		imap_iokey_spken(1);
		mute_initial = 1;
	}

	/* comment by mhfan */
	return 0;
}
#if defined (CONFIG_CPU_IMAPX200)
static int es8328_set_bias_level(struct snd_soc_codec *codec,
		enum snd_soc_bias_level level)
{
	//u16 reg = wm8731_read_reg_cache(codec, WM8731_PWR) & 0xff7f;

	return 0;
}


#else
static int es8328_dapm_event(struct snd_soc_codec *codec, int event)
{
	u16 pwr_reg = es8328_read_reg_cache(codec, es8328_PWR1) & 0xfe3e;
	pwr_reg |= 0x2;//lzcx micbias on
	switch (event) {
	case SNDRV_CTL_POWER_D0: /* full On */
		/* set vmid to 50k and unmute dac */
		es8328_write(codec, es8328_PWR1, pwr_reg | 0x00c0);
		break;
	case SNDRV_CTL_POWER_D1: /* partial On */
		es8328_write(codec, es8328_PWR1, pwr_reg | 0x01c0); 
		break;
	case SNDRV_CTL_POWER_D2: /* partial On */
		/* set vmid to 5k for quick power up */
		es8328_write(codec, es8328_PWR1, pwr_reg | 0x01c1);
		break;
	case SNDRV_CTL_POWER_D3hot: /* Off, with power */
		/* mute dac and set vmid to 500k, enable VREF */
		es8328_write(codec, es8328_PWR1, pwr_reg | 0x0141);
		break;
	case SNDRV_CTL_POWER_D3cold: /* Off, without power */
		es8328_write(codec, es8328_PWR1, 0x0001);
		break;
	}
//	codec->dapm_state = event;
	return 0;
}
#endif
#define es8328_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)

#define es8328_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE)
static struct snd_soc_dai_ops es8328_ops = {
	.hw_params = es8328_pcm_hw_params,
	.set_fmt = es8328_set_dai_fmt,
	.set_sysclk = es8328_set_dai_sysclk,
	.digital_mute = es8328_mute,
	.set_clkdiv = es8328_set_dai_clkdiv,
};


struct snd_soc_dai es8328_dai = {
	.name = "es8328",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = es8328_RATES,
		.formats = es8328_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = es8328_RATES,
		.formats = es8328_FORMATS,},
	.ops = &es8328_ops,	

#if 0
	.dai_ops = {
		.digital_mute = es8328_mute,
		.set_fmt = es8328_set_dai_fmt,
		.set_sysclk = es8328_set_dai_sysclk,
		.set_clkdiv = es8328_set_dai_clkdiv,
	},
#endif
};
EXPORT_SYMBOL_GPL(es8328_dai);

static void es8328_work(struct work_struct *work)
{
	struct snd_soc_codec *codec =
		container_of(work, struct snd_soc_codec, delayed_work.work);

	es8328_set_bias_level(codec, SND_SOC_BIAS_ON);
}

static int es8328_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	es8328_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int es8328_codec_init(struct snd_soc_codec *codec)
{
	es8328_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	schedule_delayed_work(&codec->delayed_work, msecs_to_jiffies(1000));

#ifdef	CONFIG_HHTECH_MINIPMP
	// init first mutes
	es8328_write(codec, ES8328_MASTERMODE, 0x00);		// 0x08
	es8328_write(codec, ES8328_CHIPPOWER, 0xf3);		// 0x02
	es8328_write(codec,ES8328_CONTROL1 , 0x05);			// 0x00

	es8328_write(codec, ES8328_CONTROL2, 0x040);		// 0x01, low power
//	es8328_write(codec, ES8328_CHIPLOPOW1, 0x00);		// 0x05, low power
//	es8328_write(codec, ES8328_CHIPLOPOW2, 0x00);		// 0x06 low power
//	es8328_write(codec, ES8328_ANAVOLMANAG, 0x07f);		// 0x07 low power
	es8328_write(codec, ES8328_ADCPOWER, 0x00);			// 0x03

	es8328_write(codec, ES8328_DACPOWER,  0x30);		// 0x04
	es8328_write(codec, ES8328_ADCCONTROL1,  0x77);		// 0x09
	es8328_write(codec, ES8328_ADCCONTROL2,  0xf0);		// 0x0a
	es8328_write(codec, ES8328_ADCCONTROL3,  0x82);		// 0x0b
#endif//CONFIG_HHTECH_MINIPMP

	es8328_write(codec, ES8328_ADCCONTROL4,  0x4c);		// 0x0c
	es8328_write(codec, ES8328_ADCCONTROL5,  0x02);		// 0x0d
	es8328_write(codec, ES8328_ADCCONTROL7,  0xf0);		// 0x0d

	es8328_write(codec, ES8328_ADCCONTROL8, 0x01);		// 0x10
	es8328_write(codec, ES8328_ADCCONTROL9, 0x01);		// 0x11

	es8328_write(codec, ES8328_ADCCONTROL10,  0xe2);	// 0x12
	es8328_write(codec, ES8328_ADCCONTROL11, 0xc0);		// 0x13
	es8328_write(codec, ES8328_ADCCONTROL12, 0x05);		// 0x14
	es8328_write(codec, ES8328_ADCCONTROL13, 0x06);		// 0x15
	es8328_write(codec, ES8328_ADCCONTROL14, 0xb3);		// 0x16
	es8328_write(codec, ES8328_DACCONTROL1, 0x18);		// 0x17
	es8328_write(codec, ES8328_DACCONTROL2, 0x02);		// 0x18

	es8328_write(codec, ES8328_DACCONTROL4, 0x18);		// 0x1a
	es8328_write(codec, ES8328_DACCONTROL5, 0x18);		// 0x1b

	es8328_write(codec, ES8328_DACCONTROL16, 0x09);		// 0x26
	es8328_write(codec, ES8328_DACCONTROL17, 0xb8);		// 0x27
	es8328_write(codec, ES8328_DACCONTROL18, 0x38);		// 0x28
	es8328_write(codec, ES8328_DACCONTROL19, 0x38);		// 0x29
	es8328_write(codec, ES8328_DACCONTROL20, 0xb8);		// 0x2a
	es8328_write(codec, ES8328_DACCONTROL24, 0x1a);		// 0x2e
	es8328_write(codec, ES8328_DACCONTROL25, 0x1a);		// 0x2f
	es8328_write(codec, ES8328_CHIPPOWER, 0x00);		//0x02

	return 0;
}

static int es8328_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	return es8328_codec_init(codec);
}

/*
 * initialise the es8328 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int es8328_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->card->codec;
	int reg, ret = 0;


	codec->name = "es8328";
	codec->owner = THIS_MODULE;
	codec->read = es8328_read_reg_cache;
	codec->write = es8328_write;
	codec->set_bias_level = es8328_set_bias_level;
	codec->dai = &es8328_dai;
	codec->num_dai = 1;
	codec->reg_cache = (void*)es8328_reg;
	codec->reg_cache_size = ARRAY_SIZE(es8328_reg);

#ifdef CONFIG_HHBF_FAST_REBOOT
	if (_bfin_swrst & FAST_REBOOT_FLAG) init_reboot = 1;
	// XXX: read register values from codec?
#endif

	//es8328_reset(codec);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "es8328: failed to create pcms\n");
		goto pcm_err;
	}


	es8328_codec_init(codec);
	
	es8328_add_controls(codec);
	es8328_add_widgets(codec);
//	ret = snd_soc_register_card(socdev);
	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "es8328: failed to register card\n");
		goto card_err;
	}

#ifdef	CONFIG_HHTECH_MINIPMP
	init_reboot = 0;
#endif//CONFIG_HHTECH_MINIPMP

	return ret;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
#ifndef	CONFIG_HHTECH_MINIPMP
	kfree(codec->reg_cache);
#endif//CONFIG_HHTECH_MINIPMP
	return ret;
}

/* If the i2c layer weren't so broken, we could pass this kind of data
   around */
static struct snd_soc_device *es8328_socdev;
#if 0
#ifdef	CONFIG_HHTECH_MINIPMP
void hhbf_audio_close(void)
{
    if (!es8328_socdev) return;
    cancel_delayed_work(&es8328_socdev->delayed_work);
    es8328_reset(es8328_socdev->codec);
}
EXPORT_SYMBOL(hhbf_audio_close);
#endif//CONFIG_HHTECH_MINIPMP
#endif
/*******************************************************************************************************/
#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)

static int es8328_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct snd_soc_device *socdev = es8328_socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	int ret;
	i2c_set_clientdata(i2c, codec);
	codec->control_data = i2c;
//	printk("*********es8328_i2d_probe!\n");
	ret = es8328_init(socdev);
	if (ret < 0)
		pr_err("failed to initialise WM8971\n");

	return ret;
}

static int es8328_i2c_remove(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	kfree(codec->reg_cache);
	return 0;
}

static const struct i2c_device_id es8328_i2c_id[] = {
	{ "es8328", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, es8328_i2c_id);

static struct i2c_driver es8328_i2c_driver = {
	.driver = {
		.name = "es8328 I2C Codec",
		.owner = THIS_MODULE,
	},
	.probe    = es8328_i2c_probe,
	.remove   = es8328_i2c_remove,
	.id_table = es8328_i2c_id,
};

static int es8328_add_i2c_device(struct platform_device *pdev,
				 const struct es8328_setup_data *setup)
{
	struct i2c_board_info info;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	int ret;
	ret = i2c_add_driver(&es8328_i2c_driver);
	if (ret != 0) {
		dev_err(&pdev->dev, "can't add i2c driver\n");
		return ret;
	}
	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = setup->i2c_address;
	strlcpy(info.type, "es8328", I2C_NAME_SIZE);

	adapter = i2c_get_adapter(setup->i2c_bus);
	if (!adapter) {
		dev_err(&pdev->dev, "can't get i2c adapter %d\n",
			setup->i2c_bus);
		goto err_driver;
	}
	client = i2c_new_device(adapter, &info);

	i2c_put_adapter(adapter);
	if (!client) {
		dev_err(&pdev->dev, "can't add i2c device at 0x%x\n",
			(unsigned int)info.addr);
		goto err_driver;
	}
	return 0;

err_driver:
	i2c_del_driver(&es8328_i2c_driver);
	return -ENODEV;
}

#endif
/**********************************************************************************************************/
static int es8328_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct es8328_setup_data *setup = socdev->codec_data;
	struct snd_soc_codec *codec;
	int ret = 0;
	info("Audio Codec Driver %s", es8328_VERSION);
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

#ifndef	CONFIG_HHTECH_MINIPMP
	es8328 = kzalloc(sizeof(struct es8328_priv), GFP_KERNEL);
	if (es8328 == NULL) {
		kfree(codec);
		return -ENOMEM;
	}

	codec->private_data = es8328;
#endif//CONFIG_HHTECH_MINIPMP
	socdev->card->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	es8328_socdev = socdev;
	INIT_DELAYED_WORK(&codec->delayed_work, es8328_work);

#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)
	if (setup->i2c_address) {
		codec->hw_write = (hw_write_t)i2c_master_send;
		codec->hw_read = (hw_write_t)i2c_master_recv;
		ret = es8328_add_i2c_device(pdev, setup);
	}
#else
		/* Add other interfaces here */
#endif

	return ret;
}

/*
 * This function forces any delayed work to be queued and run.
 */
static int run_delayed_work(struct delayed_work *dwork)
{
	int ret;

	/* cancel any work waiting to be queued. */
	ret = cancel_delayed_work(dwork);

	/* if there was any work waiting then we run it now and
	 * wait for it's completion */
	if (ret) {
		schedule_delayed_work(dwork, 0);
		flush_scheduled_work();
	}
	return ret;
}

/* power down chip */
static int es8328_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	if (codec->control_data)
	  es8328_set_bias_level(codec, SND_SOC_BIAS_OFF);

	run_delayed_work(&codec->delayed_work);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)
	i2c_del_driver(&es8328_i2c_driver);
#endif
#ifndef	CONFIG_HHTECH_MINIPMP
	kfree(codec->private_data);
#endif//CONFIG_HHTECH_MINIPMP
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_es8328 = {
	.probe = 	es8328_probe,
	.remove = 	es8328_remove,
	.suspend = 	es8328_suspend,
	.resume =	es8328_resume,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_es8328);

MODULE_DESCRIPTION("ASoC es8328 driver");
MODULE_AUTHOR("Liam Girdwood");
MODULE_LICENSE("GPL");
