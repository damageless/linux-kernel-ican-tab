

#include <linux/platform_device.h>
#include <linux/types.h>                                                                                                                                                
#include <linux/workqueue.h>                                                                                                                                            
#include <linux/interrupt.h>                                                                                                                                            
#include <linux/kernel.h>   
#include <linux/i2c.h>
#include <asm/irq.h>
#include <mach/imapx_gpio.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/input.h> 
#include <linux/spinlock.h>
#include <linux/gpio.h>

#include "ep0700m01.h"


struct ep0700m01{


	long x1;
	long y1;
	long x2;
	long y2;
	long figer;
	long w1;
	long w2;
	unsigned int irq;

	struct input_dev * dev;
	struct i2c_client *client;
	struct hrtimer	timer;
	struct work_struct work;
	struct workqueue_struct *queue;
	struct delayed_work delay_work;
	spinlock_t uTouch_spin;

};

struct ep0700m01 ep07;

unsigned int ep0700_int;

static int ep07_write(struct i2c_client *i2c, unsigned int reg,
        unsigned int value)
{
        u8 data[2];
        /* data is
         *   D15..D9 es8328 register offset
         *   D8...D0 register data
         */
        data[0] = reg ;
        data[1] = value;
        if (i2c_master_send(i2c, data, 2) == 2)
                return 0;
        else
        {
                printk("i2c_transfer_error!\n");
                return -EIO;
        }
//return 0;
}


static int ep07_abs_read(struct i2c_client *i2c, u8 *data, int cnt)
{
	u8 regadr;

	if(i2c_master_recv(i2c, data, cnt)!=cnt){
		printk("ep07_abs_read i2c_master_recv error\n");
		return 0;
	}		

	return 1;
}

static int ep07_figer_read(struct i2c_client *i2c, u8 *data)
{
	u8 regadr;

	regadr = 0x9;
	
	if(i2c_master_send(i2c, &regadr, 1)!=1){
		printk("ep07_figer_read i2c_master_send error\n");
		return 0;
	}

	if(i2c_master_recv(i2c, data, 1)!=1){
		printk("ep07_figer_read i2c_master_recv error\n");
		return 0;
	}

	//printk("ep07_figer_read figer is %d\n",data[0]);
	
	return 1;
}

static int ep07_read(struct i2c_client *i2c, u8 *data)
{
	int i = 0;

        i2c_master_recv(i2c, data, 11);
#if 0
	printk("multi i2c read finsih\n");
	for(i=0;i<11;i++)
	{
        	printk("i2c read data is %x\n", data[i]);
	}
#endif
	return 1;
}


static int ep07_init(struct i2c_client *i2c)
{
	u8 data[20];	
	

	ep07_read(i2c,data);

	return 0;
}


static int atmega168_read(struct i2c_client *i2c, u8 *data, int cnt)
{
	u8 regadr; 
        uint32_t i = 0;

        regadr = 0x0;

        if(i2c_master_send(i2c, &regadr, 1)!=1){
                 printk("atmega168_abs_read i2c_master_send error\n");
                 return 0;
        }
     
     
        if(i2c_master_recv(i2c, data, cnt)!=cnt){
                printk("atmega168_abs_read i2c_master_recv error\n");
                return 0;
        }

	return 1;
}


static int atmega168_abs_read(struct i2c_client *i2c, u8 *data, int cnt)
{
        u8 regadr;
	uint32_t i = 0;

	regadr = 0x2;

	if(i2c_master_send(i2c, &regadr, 1)!=1){
                 printk("atmega168_abs_read i2c_master_send error\n");
                 return 0;
        }


        if(i2c_master_recv(i2c, data, cnt)!=cnt){
                printk("atmega168_abs_read i2c_master_recv error\n");
                return 0;
        }               
       
#if 0
	printk("--------atmega168_abs_read--------\n");
	for(i=0; i<cnt ;i++)
	{
		printk("%x\n",data[i]);
	}

#endif
 
        return 1;
}       



static int atmega168_figer_read(struct i2c_client *i2c, u8 *data)
{
        u8 regadr;
	uint32_t i = 0;

        regadr = 0x0;

        if(i2c_master_send(i2c, &regadr, 1)!=1){
                printk("atmega168_figer_read i2c_master_send error\n");
                return 0;
        }

        if(i2c_master_recv(i2c, data, 1)!=1){
                printk("atmega168_figer_read i2c_master_recv error\n");
                return 0;
        }

 #if 0
         printk("--------atmega168_figer_read--------\n");
         printk("%x\n",data[0]);
 
 #endif

	
	return 1;

}
    
static int atmega168_area_read(struct i2c_client *i2c, u8 *regadr, u8 *data, int len)
{

	
	if(i2c_master_send(i2c, regadr, 1)!=1){
                 printk("atmega168_area_read i2c_master_send error\n");
                 return 0;
         }
 
         if(i2c_master_recv(i2c, data, len)!=len){
                 printk("atmega168_area_read i2c_master_recv error\n");
                 return 0;
         }

	return 1;

}

static int atmega168_firmware(struct i2c_client *i2c)
{
	uint8_t data[2];

	data[0] = 0x3A;
	data[1] = 30;

	if (i2c_master_send(i2c, data, 2) != 2)
	{
		printk("%s X_threshold err\n", __func__);
	}	
	
	data[0] = 0x3B;
	data[1] = 30;
	
	if (i2c_master_send(i2c, data, 2) != 2)
	{
		printk("%s Y_threshold err\n", __func__);
	}	
	
	printk("%s update ok\n", __func__);
}

static int atmega168_init(struct i2c_client *i2c)
{
	uint8_t data[2];

	data[0] = 0x37;
	data[1] = 0x3;

	if (i2c_master_send(i2c, data, 2) == 2)
	{
		 mdelay(1000);
#if 1
	atmega168_firmware(i2c);
#endif
                 return 0;
	}
        else
        {
                 printk("atmega168_init i2c_transfer_error!\n");
                 return -EIO;
        }

	
}

static int ep07_i2c_remove(struct i2c_client *client)
{
        return 0;
}


static int ep07_i2c_probe(struct i2c_client *i2c,
                            const struct i2c_device_id *id)
{
        int ret;

	ep07.client = i2c;

        i2c_set_clientdata(i2c, NULL);
        printk("ep07_i2c_probe!\n");
        //ret = ep07_init(i2c);
        ret = atmega168_init(i2c);
	if (ret < 0)
                pr_err("failed to initialise EP07\n");

        return ret;
}

static const struct i2c_device_id ep07_i2c_id[] = {
        { "ep07", 0 },
        { }
};
MODULE_DEVICE_TABLE(i2c, ep07_i2c_id)

static struct i2c_driver ep07_i2c_driver = {
        .driver = {
                .name = "ep07 I2C multi touch",                                                                                                                             
                .owner = THIS_MODULE,
        },      
        .probe    = ep07_i2c_probe,                                                                                                                                   
        .remove   = ep07_i2c_remove,                                                                                                                                  
        .id_table = ep07_i2c_id,                                                                                                                                      
};


static int ep07_add_i2c_device(struct platform_device *pdev,                                                                                                          
                                  const struct ep07_setup_data *setup)                                                                                                 
{                                                                                                                                                                       
        struct i2c_board_info info;                                                                                                                                     
        struct i2c_adapter *adapter;                                                                                                                                    
        struct i2c_client *client;                                                                                                                                      
        int ret;                                                                                                                                                        
        printk("ep07_add_i2c_device!\n");                                                                                                                      
        ret = i2c_add_driver(&ep07_i2c_driver);                                                                                                                       
        if (ret != 0) {                                                                                                                                                 
                dev_err(&pdev->dev, "can't add i2c driver\n");                                                                                                          
                return ret;                                                                                                                                             
        }                                                                                                                                                               
#if 1                                                                                                                                                                   
        memset(&info, 0, sizeof(struct i2c_board_info));                                                                                                                
        info.addr = setup->i2c_address;                                                                                                                                 
        strlcpy(info.type, "ep07", I2C_NAME_SIZE);                                                                                                                    
                                                                                                                                                                        
        adapter = i2c_get_adapter(setup->i2c_bus);                                                                                                                      
        if (!adapter) {                                                                                                                                                 
              dev_err(&pdev->dev, "can't get i2c adapter %d\n",                                                                                                       
                        setup->i2c_bus);                                                                                                                                
                 goto err_driver;                                                                                                                                        
        }         
	printk("i2c_new_device\n");                                                                                                                                                      
        client = i2c_new_device(adapter, &info);                                                                                                                        
                                                                                                                                                                        
        printk("es07_new_device!\n");                                                                                                                          
        i2c_put_adapter(adapter);                                                                                                                                       
        if (!client) {                                                                                                                                                  
                dev_err(&pdev->dev, "can't add i2c device at 0x%x\n",                                                                                                   
                        (unsigned int)info.addr);                                                                                                                       
                goto err_driver;                                                                                                                                        
        }
	else
	{
	//	ep07.client = client;
	}                                                                                                                                                               
#endif                                                                                                                                                                  
        return 0;                                                                                                                                                       
                                                                                                                                                                      
err_driver:                                                                                                                                                             
        i2c_del_driver(&ep07_i2c_driver);                                                                                                                             
        return -ENODEV;                                                                                                                                                 
}


static int get_pendown_state()
{
	return !(imapx_gpio_getpin(ep0700_int, IG_NORMAL));
}

static void ep07_Hotplug(struct work_struct *work)
{
	u8 data[20];

	printk("ep07_Hotplug called\n");

	ep07_read(ep07.client,data);

	printk("ep07_Hotplug end\n");

}


static void ep07_read_loop(struct work_struct *workdata)
{
	u8 data[20];	
	uint32_t tmp = 0;

	//printk("ep07_read_loop called\n");
	if(get_pendown_state())
	{

#if 0
		if(atmega168_figer_read(ep07.client,data)==0)
		{
			goto ts_iic_err;
		}		

		ep07.figer = data[0];		
#endif

                if(atmega168_read(ep07.client,data,10)==0)
                {
                        goto ts_iic_err;
                }               

                ep07.figer = data[0];           
                ep07.x1 = data[3]<<8 | data[2];
                ep07.y1 = data[5]<<8 | data[4];
                ep07.x2 = data[7]<<8 | data[6];
                ep07.y2 = data[9]<<8 | data[8];

		
		if(ep07.figer == 0)
		{
			printk("No figer is pressed\n");
			enable_irq(ep07.irq);
			input_report_abs(ep07.dev, ABS_MT_TOUCH_MAJOR, 0);
			input_mt_sync(ep07.dev);
			input_sync(ep07.dev);
			return;
		}
		if(ep07.figer == 1)
		{
#if 0
			if(atmega168_abs_read(ep07.client,data,4)==0){
				goto ts_iic_err;
			}			
			
			ep07.x1 = data[1]<<8 | data[0];
			ep07.y1 = data[3]<<8 | data[2];
			
			if( ep07.x1 == 0 && ep07.y1 == 0)
			{
				goto ts_iic_err;
			}
#endif
			//ep07.w1 = data[5];
			ep07.w1 = 10;
	
			input_report_abs(ep07.dev, ABS_MT_TOUCH_MAJOR, ep07.w1);
			input_report_abs(ep07.dev, ABS_MT_POSITION_X, ep07.x1);
			input_report_abs(ep07.dev, ABS_MT_POSITION_Y, ep07.y1);

			input_mt_sync(ep07.dev);

			input_sync(ep07.dev);

//			msleep(10);
//			printk("w1,%d\n",ep07.w1);
//			printk("one figer is pressed,%d,%d\n",ep07.x1,ep07.y1);
		}
		if(ep07.figer == 2)
		{
#if 0
			if(atmega168_abs_read(ep07.client,data,8)==0){
				goto ts_iic_err;
			}

                        ep07.x1 = data[1]<<8 | data[0];
                        ep07.y1 = data[3]<<8 | data[2];
                        ep07.x2 = data[5]<<8 | data[4];
                        ep07.y2 = data[7]<<8 | data[6];

			if(( ep07.x1 == 0 && ep07.y1 == 0) || ( ep07.x2 == 0 && ep07.y2 == 0))
			{
        			goto ts_iic_err;
			}	
#endif
			ep07.w1 = data[11];

			input_report_abs(ep07.dev, ABS_MT_TOUCH_MAJOR, 10);
			input_report_abs(ep07.dev, ABS_MT_POSITION_X, ep07.x1);
                        input_report_abs(ep07.dev, ABS_MT_POSITION_Y, ep07.y1);
                        input_mt_sync(ep07.dev);

			ep07.w2 = data[13];
			input_report_abs(ep07.dev, ABS_MT_TOUCH_MAJOR, 10);	
			input_report_abs(ep07.dev, ABS_MT_POSITION_X, ep07.x2);
                        input_report_abs(ep07.dev, ABS_MT_POSITION_Y, ep07.y2);
                        input_mt_sync(ep07.dev);

                        input_sync(ep07.dev);

		}
		

	

//	schedule_work(&ep07.work);	

	//	ep07.timer.function = ep07_timer;
	//	hrtimer_start(&ep07.timer, ktime_set(0, 20),HRTIMER_MODE_REL);
//		queue_work(ep07.queue, &ep07.work);


		queue_delayed_work(ep07.queue, &ep07.delay_work, 3);

//		schedule_delayed_work(&ep07.delay_work, 3);
  //      msleep(1);

	}

	else
	{
ts_iic_err:
		enable_irq(ep07.irq);
		input_report_abs(ep07.dev, ABS_MT_TOUCH_MAJOR, 0);

		input_mt_sync(ep07.dev);
		input_sync(ep07.dev);
		printk("figer up 1\n");
	}

}


static irqreturn_t multi_touch_irq(int irq, void *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&ep07.uTouch_spin, flags);
	if(get_pendown_state())
	{
	//	ep07.timer.function = ep07_timer;
	//	hrtimer_start(&ep07.timer, ktime_set(0, 20),HRTIMER_MODE_REL);
		disable_irq_nosync(ep07.irq);
	//	queue_work(ep07.queue, &ep07.work);

		queue_delayed_work(ep07.queue, &ep07.delay_work, 0);

//		schedule_delayed_work(&ep07.delay_work, 3);

	}
	//printk("multi_touch_irq end\n");
	spin_unlock_irqrestore(&ep07.uTouch_spin, flags);	

	return IRQ_HANDLED;
}


static int __init multi_touch_probe(struct platform_device *pdev)
{
	int ret = 0;
	//int irq = 0;
	int err = 0;
	volatile int tmp = 0;

	struct ep07_setup_data setup;

	printk("multi_touch_probe called\n");

	platform_get_drvdata(pdev);

	ep07.dev = input_allocate_device();

#if 0
	__set_bit(EV_ABS, ep07.dev->evbit);
	__set_bit(EV_SYN, ep07.dev->evbit);
	

	input_set_abs_params(ep07.dev, ABS_MT_TOUCH_MAJOR, 0, 1000, 0, 0);
	input_set_abs_params(ep07.dev, ABS_MT_POSITION_X, 0, 0x4dff, 0, 0);
	input_set_abs_params(ep07.dev, ABS_MT_POSITION_Y, 0, 0x2bff, 0, 0);
#endif


	__set_bit(EV_ABS, ep07.dev->evbit);
        __set_bit(EV_SYN, ep07.dev->evbit);
	__set_bit(EV_KEY, ep07.dev->evbit);

	input_set_abs_params(ep07.dev, ABS_MT_TOUCH_MAJOR, 0, 256, 0, 0);
	input_set_abs_params(ep07.dev, ABS_MT_WIDTH_MAJOR, 0, 256, 0, 0);

	input_set_abs_params(ep07.dev, ABS_PRESSURE, 0, 256, 0, 0);
	input_set_abs_params(ep07.dev, ABS_MT_POSITION_X,
                                        0, 0x3c00, 0, 0);

	input_set_abs_params(ep07.dev, ABS_MT_POSITION_Y,
                                        0, 0x2400, 0, 0);


	ret = input_register_device(ep07.dev);
	if(ret){
		printk(KERN_ERR "%s: unabled to register input device, ret = %d\n",__FUNCTION__, ret);
		return ret;
	}

	ep07.uTouch_spin= SPIN_LOCK_UNLOCKED;	
	
//	INIT_WORK(&ep07.work, ep07_read_loop);
	INIT_DELAYED_WORK(&ep07.delay_work, ep07_read_loop);
	
	ep07.queue = create_singlethread_workqueue("ep07-touch-screen-read-loop");

//	hrtimer_init(&ep07.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
//	ep07.timer.function = ep07_timer;

	printk("multi_touch_probe called\n");	
	setup.i2c_bus = CONFIG_CTP_EP0700_I2C + 1;
	setup.i2c_address = (0x5C);

	ep07.dev->name = "ep07-touch";

	ret = ep07_add_i2c_device(ep07.dev, &setup);

	ep0700_int = __imapx_name_to_gpio(CONFIG_CTP_EP0700_INT);
	if(ep0700_int == IMAPX_GPIO_ERROR) {
		printk(KERN_ERR "failed to get ep0700_int pin.\n");
		return -1;
	}

	ep07.irq = imapx_gpio_to_irq(ep0700_int);
	//irq = IRQ_EINT1;//platform_get_irq(ep07.dev, 0);

	printk("multi touch screen irq is %d\n",ep07.irq);
	if (ep07.irq < 0) {
                dev_err(&pdev->dev, "No IRQ specified\n");
                err = -ENOENT;
                goto err_no_irq;
	}

	err = request_irq(ep07.irq, multi_touch_irq, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING |IRQF_DISABLED, ep07.dev->name, ep07.dev);
	if (err) {
                dev_err(&pdev->dev, "Cannot claim IRQ\n");
                goto err_no_irq;
        }

	imapx_gpio_setirq(ep0700_int, FILTER_MAX, IG_FALL, 1);

	/*
	tmp = __raw_readl(rEINTCON);
	tmp &= ~(0x7<<20);
	tmp |= (0x2<<20);// FALL EDGE
	__raw_writel(tmp,rEINTCON);

	tmp = __raw_readl(rEINTFLTCON1);
	tmp &= ~(0xff<<8);
	tmp |= (0x1<<15) | (0x10<<8);
	__raw_writel(tmp,rEINTFLTCON1);
	*/

	/*
	 *
	 * for test
	 */
#if 0
	{
		uint8_t date ;
		uint8_t ad1 = 0x7d, ad2 = 0x3d;
		uint8_t buf[64];
		int i =0;

		ep07_write(ep07.client, 0x37, 0x03);
		mdelay(200);
		ep07_write(ep07.client, 0xbd, 0x00);

		ep07_write(ep07.client, 0x37, 0x01);
		ep07_write(ep07.client, 0x40, 0x00);
		ep07_abs_read(ep07.client,&date, 1);
		printk(KERN_INFO "get date0 %x\n",date);

		ep07_write(ep07.client, 0x37, 0x01);
		ep07_write(ep07.client, 0xd3, 0x00);
		ep07_abs_read(ep07.client,&date, 1);
		printk(KERN_INFO "get date1 %x\n",date);

		ep07_write(ep07.client, 0x37, 0x01);
		ep07_write(ep07.client, 0x66, 0x01);
		ep07_abs_read(ep07.client,&date, 1);
		printk(KERN_INFO "get date2 %x\n",date);

		ep07_write(ep07.client, 0xc2, 0x00);
		
		for(i = 0 ;i < 64; i++)
		{
			buf[i] = 0;
		}

		if(i2c_master_send(ep07.client, &ad1, 1)!=1){                  
			printk("ep07_figer_read i2c_master_send error\n");
			return 0;
		}
		if(i2c_master_recv(ep07.client, buf, 64)!=64){
			printk("ep07_0x3d_read i2c_master_recv error\n");
			return 0;
		}                                                         
		for(i = 0 ;i < 64; i++)
		{
			printk(KERN_INFO "ad1 buf[%d]-%x\n",i,buf[i]);
		}

		for(i = 0 ;i < 64; i++)
		{
			buf[i] = 0;
		}

		if(i2c_master_send(ep07.client, &ad2, 1)!=1){                  
			printk("ep07_figer_read i2c_master_send error\n");
			return 0;
		}
		if(i2c_master_recv(ep07.client, buf, 64)!=64){
			printk("ep07_0x3d_read i2c_master_recv error\n");
			return 0;
		}                                                         
		for(i = 0 ;i < 64; i++)
		{
			printk(KERN_INFO "ad2 buf[%d]-%x\n",i,buf[i]);
		}



	}
#endif
	//while(1);
#if 0
	battery_val = 1000;
#endif
	return ret;

err_free_gpio:
        //if (ts->gpio_pendown != -1)
         //       gpio_free(ts->gpio_pendown);

err_no_irq:

//	while(1);
	return err;

}


static int __exit multi_touch_remove(struct platform_device *dev)
{

        return 0;
}



static int multi_touch_suspend(struct platform_device *pdev, pm_message_t msg)
{
	printk("multi_touch_suspend called\n");
        return 0;
}

static int multi_touch_resume(struct platform_device *pdev)
{
	printk("multi_touch_resume called\n");
        return 0;
}


MODULE_ALIAS("platform:multi touchscreen");
static struct platform_driver multi_touch_driver = {
         .remove         = __exit_p(multi_touch_remove),
         .suspend        = multi_touch_suspend,
         .resume         = multi_touch_resume,
         .driver         = {
                 .name   = "imapx200_multi_touch",
                 .owner  = THIS_MODULE,
         },
};



static int __init ep07_touch_init()
{

	printk("ep07_touch_init called \n");
	return platform_driver_probe(&multi_touch_driver, multi_touch_probe);
} 


static void __exit ep07_touch_exit()
{
	platform_driver_unregister(&multi_touch_driver);
}


module_init(ep07_touch_init);
module_exit(ep07_touch_exit);

MODULE_DESCRIPTION("TouchScreen  Driver");
MODULE_AUTHOR("Jay Hu, <Jay.hu@infotmic.com.cn>");
MODULE_LICENSE("GPL");



