#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <asm/io.h>


#include <linux/gpio.h>
#include <mach/hardware.h>
#include <plat/board.h>
#include <plat/cpu.h>

#include <linux/input/mt.h>
#include <linux/slab.h>


#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif /* CONFIG_HAS_EARLYSUSPEND */

#define SSD_ICS_SLOT_REPORT 1

#define CONFIG_TOUCHSCREEN_SSL_DEBUG	
#undef  CONFIG_TOUCHSCREEN_SSL_DEBUG

#define DEVICE_ID_REG                 2
#define EVENT_STATUS                  0x79
#define EVENT_MSK_REG                 0x7A
#define IRQ_MSK_REG                   0x7B
#define FINGER01_REG                  0x7C
#define FINGER02_REG                  0x7D
#define FINGER03_REG                  0x7E
#define FINGER04_REG                  0x7F

int Ssd_Timer1,Ssd_Timer2,Ssd_Timer_flag;

/* TODO: Re-check this on next board iteration */
#define		ON_TOUCH_INT	(20)

struct ChipSetting {
	char No;
	char Reg;
	char Data1;
	char Data2;
};

#include "ssd2543-ts_TP.h"  

void deviceReset(struct i2c_client *client); 
void deviceResume(struct i2c_client *client);
void deviceSuspend(struct i2c_client *client);
void SSD253xdeviceInit(struct i2c_client *client); 

static int ssd253x_ts_open(struct input_dev *dev);
static void ssd253x_ts_close(struct input_dev *dev);
static int ssd253x_ts_suspend(struct i2c_client *client, pm_message_t mesg);
static int ssd253x_ts_resume(struct i2c_client *client);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void ssd253x_ts_early_suspend(struct early_suspend *h);
static void ssd253x_ts_late_resume(struct early_suspend *h);
#endif /* CONFIG_HAS_EARLYSUSPEND */

static enum hrtimer_restart ssd253x_ts_timer(struct hrtimer *timer);
static irqreturn_t ssd253x_ts_isr(int irq, void *dev_id);
static struct workqueue_struct *ssd253x_wq;


struct ssl_ts_priv {
	struct i2c_client *client;
	struct input_dev *input;
	struct hrtimer timer;
	struct work_struct  ssl_work;
#ifdef	CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif 

	int irq;
	int use_irq;
	int FingerNo;
	int FingerX[FINGERNO];
	int FingerY[FINGERNO];
	int FingerP[FINGERNO];

	int Resolution;
	int EventStatus;
	int FingerDetect;

	int sFingerX[FINGERNO];
	int sFingerY[FINGERNO];
	int pFingerX[FINGERNO];
	int pFingerY[FINGERNO];
};

int ReadRegister(struct i2c_client *client,uint8_t reg,int ByteNo)
{
	unsigned char buf[4];
	struct i2c_msg msg[2];
	int ret;

	memset(buf, 0xFF, sizeof(buf));
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = ByteNo;
	msg[1].buf = buf;

	ret = i2c_transfer(client->adapter, msg, 2);

	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	if(ret<2)	printk("		ReadRegister: i2c_transfer Error ! (%d)\n", ret);
	else		printk("		ReadRegister: i2c_transfer OK !\n");
	#endif
	//printk("buf[0]=0x%02x,buf[1]=0x%02x,buf[2]=0x%02x,buf[3]=0x%02x\n",buf[0],buf[1],buf[2],buf[3]);
	if(ByteNo==1) return (int)((unsigned int)buf[0]<<0);
	if(ByteNo==2) return (int)((unsigned int)buf[1]<<0)|((unsigned int)buf[0]<<8);
	if(ByteNo==3) return (int)((unsigned int)buf[2]<<0)|((unsigned int)buf[1]<<8)|((unsigned int)buf[0]<<16);
	if(ByteNo==4) return (int)((unsigned int)buf[3]<<0)|((unsigned int)buf[2]<<8)|((unsigned int)buf[1]<<16)|(buf[0]<<24);
	
	return 0;
}

void WriteRegister(struct i2c_client *client,uint8_t Reg,unsigned char Data1,unsigned char Data2,int ByteNo)
{	
	struct i2c_msg msg;
	unsigned char buf[4];
	int ret;

	buf[0]=Reg;
	buf[1]=Data1;
	buf[2]=Data2;
	buf[3]=0;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = ByteNo+1;
	msg.buf = (char *)buf;
	ret = i2c_transfer(client->adapter, &msg, 1);

	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	if(ret<0)	printk("		WriteRegister: i2c_master_send Error !\n");
	else		printk("		WriteRegister: i2c_master_send OK !\n");
	#endif
}

void SSD253xdeviceInit(struct i2c_client *client)
{	
	int i;
	for(i=0;i<sizeof(ssd253xcfgTable)/sizeof(ssd253xcfgTable[0]);i++)
	{
		WriteRegister(	client,ssd253xcfgTable[i].Reg,
				ssd253xcfgTable[i].Data1,ssd253xcfgTable[i].Data2,
				ssd253xcfgTable[i].No);
	}
	mdelay(200);
}

void deviceReset(struct i2c_client *client)
{	
	int i;
	for(i=0;i<sizeof(Reset)/sizeof(Reset[0]);i++)
	{
		WriteRegister(	client,Reset[i].Reg,
				Reset[i].Data1,Reset[i].Data2,
				Reset[i].No);
	}
	mdelay(200);   
}

void deviceResume(struct i2c_client *client)
{	
	int i;
	for(i=0;i<sizeof(Resume)/sizeof(Resume[0]);i++)
	{
		WriteRegister(	client,Resume[i].Reg,
				Resume[i].Data1,Resume[i].Data2,
				Resume[i].No);
		mdelay(200); 
	}

}

void deviceSuspend(struct i2c_client *client)
{	
	int i;
	WriteRegister(	client,Suspend[0].Reg,
				Suspend[0].Data1,Suspend[0].Data2,
				Suspend[0].No);
	mdelay(100);
	
	WriteRegister(	client,Suspend[1].Reg,
				Suspend[1].Data1,Suspend[1].Data2,
				Suspend[1].No);
	mdelay(200);
	
}


static void ssd253x_ts_work(struct work_struct *work)
{
	int i;
	unsigned short xpos=0, ypos=0, width=0;
	int FingerInfo;
	int EventStatus;
	int FingerX[FINGERNO];
	int FingerY[FINGERNO];
	int FingerP[FINGERNO];
	int clrFlag=0;
	int Ssd_Timer;

	struct ssl_ts_priv *ssl_priv = container_of(work,struct ssl_ts_priv,ssl_work);
	//printk("%s\n",__FUNCTION__);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_work!                  |\n");
	printk("+-----------------------------------------+\n");
	#endif



	EventStatus = ReadRegister(ssl_priv->client,EVENT_STATUS,2)>>4;
	//printk("EventStatus=%d[0x%x]\n",EventStatus,EventStatus);
	ssl_priv->FingerDetect=0;
	for(i=0;i<ssl_priv->FingerNo;i++)
	{
		if((EventStatus>>i)&0x1)
		{
			FingerInfo=ReadRegister(ssl_priv->client,FINGER01_REG+i,4);
			ypos = ((FingerInfo>>0)&0xF00)|((FingerInfo>>16)&0xFF);
			xpos = ((FingerInfo>>4)&0xF00)|((FingerInfo>>24)&0xFF);
			width= (FingerInfo&0x00FF);

			if(xpos!=0xFFF)
			{
				ssl_priv->FingerDetect++;

			}
			else 
			{
				EventStatus=EventStatus&~(1<<i);
				clrFlag=1;
			}
		}
		else
		{
			xpos=ypos=0xFFF;
			width=0;
			clrFlag=1;
		}
		FingerX[i]=xpos;
		FingerY[i]=ypos;
		FingerP[i]=width;
	}

	if(ssl_priv->use_irq==1) enable_irq(ssl_priv->irq);
	if(ssl_priv->use_irq==2)
	{
		if(ssl_priv->FingerDetect==0) enable_irq(ssl_priv->irq);
		else{ 
			hrtimer_start(&ssl_priv->timer, ktime_set(0, MicroTimeTInterupt), HRTIMER_MODE_REL);
			//printk("ssl_priv->FingerDetect=%d\n",ssl_priv->FingerDetect);
		}
	}

	for(i=0;i<ssl_priv->FingerNo;i++)
	{
		xpos=FingerX[i];
		ypos=FingerY[i];
		width=FingerP[i];

		if(xpos!=0xFFF)
		{

			input_mt_slot(ssl_priv->input, i);
			input_report_abs(ssl_priv->input, ABS_MT_TRACKING_ID, i);
			input_report_abs(ssl_priv->input, ABS_MT_POSITION_X, xpos);
			input_report_abs(ssl_priv->input, ABS_MT_POSITION_Y, ypos);
			
		}
		else if(ssl_priv->FingerX[i]!=0xFFF)
		{
			//ssl_priv->FingerP[i]=0;
			input_mt_slot(ssl_priv->input, i);
			input_report_abs(ssl_priv->input, ABS_MT_TRACKING_ID, -1);

		}
		if(xpos!=0xfff&&ypos!=0xfff)
		{
		 ;// printk("sd253x_ts_work: X = %d , Y = %d, W = %d\n",xpos,ypos,width);
		}
		ssl_priv->FingerX[i]=FingerX[i];
		ssl_priv->FingerY[i]=FingerY[i];
		ssl_priv->FingerP[i]=width;
	}		
	ssl_priv->EventStatus=EventStatus;
		
	input_sync(ssl_priv->input);
}

static int ssd253x_ts_probe(struct i2c_client *client,const struct i2c_device_id *idp)
{
	struct ssl_ts_priv *ssl_priv;
	struct input_dev *ssl_input;
	int error;
	int i,ret;

	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_probe!                 |\n");
	printk("+-----------------------------------------+\n");
	#endif
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: need I2C_FUNC_I2C\n");
		#endif
		return -ENODEV;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: i2c Check OK!\n");
		printk("		ssd253x_ts_probe: i2c_client name : %s\n",client->name);
		#endif
	}

	ssl_priv = kzalloc(sizeof(*ssl_priv), GFP_KERNEL);
	if (!ssl_priv)
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: kzalloc Error!\n");
		#endif
		error=-ENODEV;
		goto	err0;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: kzalloc OK!\n");
		#endif
	}
	dev_set_drvdata(&client->dev, ssl_priv);
	
	ssl_input = input_allocate_device();
	if (!ssl_input)
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: input_allocate_device Error\n");
		#endif
		error=-ENODEV;
		goto	err1;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: input_allocate_device OK\n");
		#endif
	}


	ssl_input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) | BIT_MASK(EV_SYN) ;
#if SSD_ICS_SLOT_REPORT	
    __set_bit(INPUT_PROP_DIRECT, ssl_input->propbit);
    input_mt_init_slots(ssl_input, 255);
#else	
	ssl_input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) | BIT_MASK(BTN_2);
#endif	
	ssl_input->name = client->name;
	ssl_input->id.bustype = BUS_I2C;
	ssl_input->id.vendor  = 0x2878; // Modify for Vendor ID
	ssl_input->dev.parent = &client->dev;
	ssl_input->open = ssd253x_ts_open;
	ssl_input->close = ssd253x_ts_close;

	input_set_drvdata(ssl_input, ssl_priv);
	ssl_priv->client = client;
	ssl_priv->input = ssl_input;
	ssl_priv->use_irq = ENABLE_INT;
	ssl_priv->irq = ON_TOUCH_INT;
	ssl_priv->FingerNo=FINGERNO;
	ssl_priv->Resolution=64;

	for(i=0;i<ssl_priv->FingerNo;i++)
	{
		ssl_priv->FingerP[i]=0;
		// For Finger Check Swap
		ssl_priv->sFingerX[i]=0xFFF;
		ssl_priv->sFingerY[i]=0xFFF;

		// For Adaptive Running Average
		ssl_priv->pFingerX[i]=0xFFF;
		ssl_priv->pFingerY[i]=0xFFF;
	}



	deviceReset(client);
	printk("SSL Touchscreen I2C Address: 0x%02X\n",client->addr);
	ssl_input->id.product = ReadRegister(client, DEVICE_ID_REG,2);
	printk("SSL Touchscreen Device ID  : 0x%04X\n",ssl_input->id.product);

	SSD253xdeviceInit(client);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("		ssd253X_ts_probe: %04X deviceInit OK!\n",ssl_input->id.product);
	#endif

	if(ssl_input->id.product != 0x2543)
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253X_ts_probe: %04X deviceID is invalid\n",ssl_input->id.product);
		#endif
		error=-ENODEV;
		goto	err1;
	}


	printk("++++++++++++++++++++++++++++++++++++++++++++++\n");
	__set_bit(EV_ABS, ssl_input->evbit);
	
	__set_bit(INPUT_PROP_DIRECT, ssl_input->propbit);
	set_bit(ABS_MT_POSITION_X, ssl_input->absbit);
	set_bit(ABS_MT_POSITION_Y, ssl_input->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, ssl_input->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, ssl_input->absbit);

	input_mt_init_slots(ssl_input, 5);

	input_set_abs_params(ssl_input,ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(ssl_input,ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(ssl_input,ABS_MT_TOUCH_MAJOR, 0, 15, 0, 0);
	input_set_abs_params(ssl_input,ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);

		printk("-----------------------------------------\n");

	/*input_set_abs_params(ssl_input, ABS_MT_TRACKING_ID, 0,16, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_WIDTH_MAJOR, 0, 8, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_POSITION_X,  0,SCREEN_MAX_X+1, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_POSITION_Y,  0,SCREEN_MAX_Y+1, 0, 0);*/

	#ifdef SSD253x_TOUCH_KEY
	set_bit(KEY_MENU, ssl_input->keybit);
	set_bit(KEY_HOME, ssl_input->keybit);
	set_bit(KEY_BACK, ssl_input->keybit);
	set_bit(KEY_SEARCH, ssl_input->keybit);
	#endif

	INIT_WORK(&ssl_priv->ssl_work, ssd253x_ts_work);
	error = input_register_device(ssl_input);
	if(error)
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: input_register_device input Error!\n");
		#endif
		error=-ENODEV;
		goto	err1;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: input_register_device input OK!\n");
		#endif
	}

	if((ssl_priv->use_irq==1)||(ssl_priv->use_irq==2))
	{
		// Options for different interrupt system 
	       //error = request_irq(ssl_priv->irq, ssd253x_ts_isr, IRQF_DISABLED|IRQF_TRIGGER_FALLING, client->name,ssl_priv);
		//error = request_irq(ssl_priv->irq, ssd253x_ts_isr, IRQF_TRIGGER_FALLING, client->name,ssl_priv);
		//printk("use_irq=%d\n",ssl_priv->use_irq);
		
		ret = gpio_request(ON_TOUCH_INT, "Ssd253x_irq");		
		if(ret != 0){			
			gpio_free(ON_TOUCH_INT);			
			printk("%s: ssd253x_ts irq request err\n", __FUNCTION__);		
		}		
		else{			
			gpio_direction_input(ON_TOUCH_INT);			
		}
		
		ssl_priv->irq = gpio_to_irq(ON_TOUCH_INT);
		//error = request_irq(ssl_priv->irq, ssd253x_ts_isr, IRQF_TRIGGER_LOW, client->name,ssl_priv);
		error = request_irq(ssl_priv->irq, ssd253x_ts_isr, IRQF_TRIGGER_FALLING, client->name,ssl_priv);  //add by hjc
		if(error)
		{
			#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
			printk("		ssd253x_ts_probe: request_irq Error!\n");
			#endif
			error=-ENODEV;
			goto err2;
		}
		else
		{
			#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
			printk("		ssd253x_ts_probe: request_irq OK!\n");
			#endif
		}	
	}

	if((ssl_priv->use_irq==0)||(ssl_priv->use_irq==2))
	{
		hrtimer_init(&ssl_priv->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ssl_priv->timer.function = ssd253x_ts_timer;
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: timer_init OK!\n");
		#endif
	}


#ifdef	CONFIG_HAS_EARLYSUSPEND
	ssl_priv->early_suspend.suspend = ssd253x_ts_early_suspend;
	ssl_priv->early_suspend.resume  = ssd253x_ts_late_resume;
	ssl_priv->early_suspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN-2;
	register_early_suspend(&ssl_priv->early_suspend);
#endif 
	return 0;

err2:	input_unregister_device(ssl_input);
err1:	input_free_device(ssl_input);
	kfree(ssl_priv);
err0:	dev_set_drvdata(&client->dev, NULL);
	return error;
}

static int ssd253x_ts_open(struct input_dev *dev)
{
	struct ssl_ts_priv *ssl_priv = input_get_drvdata(dev);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_open!                  |\n");
	printk("+-----------------------------------------+\n");
	#endif	
	
	deviceReset(ssl_priv->client);
	SSD253xdeviceInit(ssl_priv->client);

	deviceResume(ssl_priv->client);
	printk("%s\n",__FUNCTION__);
//HJC ADD	
	if(ssl_priv->use_irq) enable_irq(ssl_priv->irq);
	else hrtimer_start(&ssl_priv->timer, ktime_set(0, 100000000UL), HRTIMER_MODE_REL);
	return 0;
}

static void ssd253x_ts_close(struct input_dev *dev)
{
	struct ssl_ts_priv *ssl_priv = input_get_drvdata(dev);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_close!                 |\n");
	printk("+-----------------------------------------+\n");
	#endif
	deviceSuspend(ssl_priv->client);
	if((ssl_priv->use_irq==0)||(ssl_priv->use_irq==2)) hrtimer_cancel(&ssl_priv->timer);
//	if((ssl_priv->use_irq==1)||(ssl_priv->use_irq==2)) free_irq(ssl_priv->irq, ssl_priv);
}

static int ssd253x_ts_resume(struct i2c_client *client)
{
	return 0;
	struct ssl_ts_priv *ssl_priv = dev_get_drvdata(&client->dev);

	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_resume!                |\n");
	printk("+-----------------------------------------+\n");
	#endif
		Ssd_Timer_flag = 0;
		deviceResume(client);	
	
	if(ssl_priv->use_irq) enable_irq(ssl_priv->irq);
	else hrtimer_start(&ssl_priv->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	return 0;
}

static int ssd253x_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;

	struct ssl_ts_priv *ssl_priv = dev_get_drvdata(&client->dev);
	printk("%s,ssl_priv->use_irq=%d\n",__FUNCTION__,ssl_priv->use_irq);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_suspend!               |\n");
	printk("+-----------------------------------------+\n");
	#endif	

	deviceSuspend(client);

	disable_irq(ssl_priv->irq);  //add by hjc////
	cancel_work_sync(&ssl_priv->ssl_work);//add by hjc
	if((ssl_priv->use_irq==0)||(ssl_priv->use_irq==2)) hrtimer_cancel(&ssl_priv->timer);
	if((ssl_priv->use_irq==1)||(ssl_priv->use_irq==2)) disable_irq(ssl_priv->irq);//disable_irq(ssl_priv->irq, ssl_priv);	
	return 0;
}

#ifdef	CONFIG_HAS_EARLYSUSPEND
static void ssd253x_ts_late_resume(struct early_suspend *h)
{
	struct ssl_ts_priv *ssl_priv = container_of(h, struct ssl_ts_priv, early_suspend);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_late_resume!           |\n");
	printk("+-----------------------------------------+\n");
	#endif
	ssd253x_ts_resume(ssl_priv->client);
}
static void ssd253x_ts_early_suspend(struct early_suspend *h)
{
	struct ssl_ts_priv *ssl_priv = container_of(h, struct ssl_ts_priv, early_suspend);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_early_suspend!         |\n");
	printk("+-----------------------------------------+\n");
	#endif
	ssd253x_ts_suspend(ssl_priv->client, PMSG_SUSPEND);
}
#endif

static int ssd253x_ts_remove(struct i2c_client *client)
{
	struct ssl_ts_priv *ssl_priv = dev_get_drvdata(&client->dev);	
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_remove !               |\n");
	printk("+-----------------------------------------+\n");
	#endif	
	if((ssl_priv->use_irq==0)||(ssl_priv->use_irq==2)) hrtimer_cancel(&ssl_priv->timer);
	//if((ssl_priv->use_irq==1)||(ssl_priv->use_irq==2)) free_irq(ssl_priv->irq, ssl_priv);
	input_unregister_device(ssl_priv->input);
	input_free_device(ssl_priv->input);
	kfree(ssl_priv);
	dev_set_drvdata(&client->dev, NULL);
	return 0;
}

static irqreturn_t ssd253x_ts_isr(int irq, void *dev_id)
{
	struct ssl_ts_priv *ssl_priv = dev_id;
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_isr!                   |\n");
	printk("+-----------------------------------------+\n");
	#endif	
	//printk("%s\n",__FUNCTION__);
	disable_irq_nosync(ssl_priv->irq);
	queue_work(ssd253x_wq, &ssl_priv->ssl_work);
	return IRQ_HANDLED;
}

static enum hrtimer_restart ssd253x_ts_timer(struct hrtimer *timer)
{
	struct ssl_ts_priv *ssl_priv = container_of(timer, struct ssl_ts_priv, timer);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_timer!                 |\n");
	printk("+-----------------------------------------+\n");
	#endif
	queue_work(ssd253x_wq, &ssl_priv->ssl_work);
	if(ssl_priv->use_irq==0) hrtimer_start(&ssl_priv->timer, ktime_set(0, MicroTimeTInterupt), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static const struct i2c_device_id ssd253x_ts_id[] = {
	{ "ssd2543-ts", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ssd253x_ts_id);

static struct i2c_driver ssd253x_ts_driver = {
	.driver = {
		.name = "ssd2543-ts",
	},
	.probe = ssd253x_ts_probe,
	.remove = ssd253x_ts_remove,
#ifndef	CONFIG_HAS_EARLYSUSPEND
	.suspend = ssd253x_ts_suspend,
	.resume = ssd253x_ts_resume,
#endif
	.id_table = ssd253x_ts_id,
};

static char banner[] __initdata = KERN_INFO "SSL Touchscreen driver, (c) 2011 Solomon Systech Ltd.\n";
static int __init ssd253x_ts_init(void)
{
	int ret;
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG	
	printk("+-----------------------------------------+\n");
	printk("|	SSL_ts_init!                      |\n");
	printk("+-----------------------------------------+\n");
	#endif
	printk(banner);
	ssd253x_wq = create_singlethread_workqueue("ssd253x_wq");
	if (!ssd253x_wq)
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_init: create_singlethread_workqueue Error!\n");
		#endif
		return -ENOMEM;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_init: create_singlethread_workqueue OK!\n");
		#endif
	}
	ret=i2c_add_driver(&ssd253x_ts_driver);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	if(ret) printk("		ssd253x_ts_init: i2c_add_driver Error! \n");
	else    printk("		ssd253x_ts_init: i2c_add_driver OK! \n");
	#endif
	return ret;
}

static void __exit ssd253x_ts_exit(void)
{
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_exit!                  |\n");
	printk("+-----------------------------------------+\n");
	#endif
	i2c_del_driver(&ssd253x_ts_driver);
	if (ssd253x_wq) destroy_workqueue(ssd253x_wq);
}

module_init(ssd253x_ts_init);
module_exit(ssd253x_ts_exit);

MODULE_AUTHOR("Solomon Systech Ltd - Design Technology, Icarus Choi");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ssd253x Touchscreen Driver 1.3");

