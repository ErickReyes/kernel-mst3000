/*
 * TI Touch Screen driver
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/input/ti_tsc.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/kthread.h>				/*for kernel_thread*/
#include <linux/sched.h>				/*for tsak_struct*/
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <mach/board-am335xevm.h>

#ifndef GPIO_TO_PIN
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))
#endif

#define DC_ADAPTER_STATUS 	GPIO_TO_PIN(2, 1)
#define CHARGE_STATUS 		GPIO_TO_PIN(1, 27)



#define TSCADC_REG_IRQEOI		0x020
#define TSCADC_REG_RAWIRQSTATUS		0x024
#define TSCADC_REG_IRQSTATUS		0x028
#define TSCADC_REG_IRQENABLE		0x02C
#define TSCADC_REG_IRQCLR		0x030
#define TSCADC_REG_IRQWAKEUP		0x034
#define TSCADC_REG_CTRL			0x040
#define TSCADC_REG_ADCFSM		0x044
#define TSCADC_REG_CLKDIV		0x04C
#define TSCADC_REG_SE			0x054
#define TSCADC_REG_IDLECONFIG		0x058
#define TSCADC_REG_CHARGECONFIG		0x05C
#define TSCADC_REG_CHARGEDELAY		0x060
#define TSCADC_REG_STEPCONFIG(n)	(0x64 + ((n-1) * 8))
#define TSCADC_REG_STEPDELAY(n)		(0x68 + ((n-1) * 8))
#define TSCADC_REG_STEPCONFIG13		0x0C4
#define TSCADC_REG_STEPDELAY13		0x0C8
#define TSCADC_REG_STEPCONFIG14		0x0CC
#define TSCADC_REG_STEPDELAY14		0x0D0
#define TSCADC_REG_FIFO0CNT		0xE4
#define TSCADC_REG_FIFO0THR		0xE8
#define TSCADC_REG_FIFO1CNT		0xF0
#define TSCADC_REG_FIFO1THR		0xF4
#define TSCADC_REG_FIFO0		0x100
#define TSCADC_REG_FIFO1		0x200
/*	Register Bitfields	*/
#define TSCADC_IRQWKUP_ENB		BIT(0)
#define TSCADC_IRQWKUP_DISABLE		0x00
//define TSCADC_STPENB_STEPENB		0x7FFF
#define TSCADC_STPENB_STEPENB		0x0002
#define TSCADC_IRQENB_FIFO0THRES	BIT(2)
#define TSCADC_IRQENB_FIFO1THRES	BIT(5)
#define TSCADC_IRQENB_PENUP		BIT(9)
#define TSCADC_IRQENB_HW_PEN		BIT(0)
#define TSCADC_STEPCONFIG_MODE_HWSYNC	0x2
#define TSCADC_STEPCONFIG_MODE_SWSYNC_CONT	0x1
#define TSCADC_STEPCONFIG_2SAMPLES_AVG	(1 << 4)
#define TSCADC_STEPCONFIG_XPP		BIT(5)
#define TSCADC_STEPCONFIG_XNN		BIT(6)
#define TSCADC_STEPCONFIG_YPP		BIT(7)
#define TSCADC_STEPCONFIG_YNN		BIT(8)
#define TSCADC_STEPCONFIG_XNP		BIT(9)
#define TSCADC_STEPCONFIG_YPN		BIT(10)
#define TSCADC_STEPCONFIG_RFP		(1 << 12)
#define TSCADC_STEPCONFIG_INM		(1 << 18)
#define TSCADC_STEPCONFIG_INP_4		(1 << 19)
#define TSCADC_STEPCONFIG_INP		(1 << 20)
#define TSCADC_STEPCONFIG_INP_5		(1 << 21)
#define TSCADC_STEPCONFIG_FIFO1		(1 << 26)
#define TSCADC_STEPCONFIG_IDLE_INP	(1 << 22)
//#define TSCADC_STEPCONFIG_OPENDLY	0x018
#define TSCADC_STEPCONFIG_OPENDLY	0x3FFFF
#define TSCADC_STEPCONFIG_SAMPLEDLY	0x88
#define TSCADC_STEPCONFIG_Z1		(3 << 19)
#define TSCADC_STEPCHARGE_INM_SWAP	BIT(16)
#define TSCADC_STEPCHARGE_INM		BIT(15)
#define TSCADC_STEPCHARGE_INP_SWAP	BIT(20)
#define TSCADC_STEPCHARGE_INP		BIT(19)
#define TSCADC_STEPCHARGE_RFM		(1 << 23)
#define TSCADC_STEPCHARGE_DELAY		0x1
#define TSCADC_CNTRLREG_TSCSSENB	BIT(0)
#define TSCADC_CNTRLREG_STEPID		BIT(1)
#define TSCADC_CNTRLREG_STEPCONFIGWRT	BIT(2)
#define TSCADC_CNTRLREG_TSCENB		BIT(7)
#define TSCADC_CNTRLREG_4WIRE		(0x1 << 5)
#define TSCADC_CNTRLREG_5WIRE		(0x1 << 6)
#define TSCADC_CNTRLREG_8WIRE		(0x3 << 5)
#define TSCADC_ADCFSM_STEPID		0x10
#define TSCADC_ADCFSM_FSM		BIT(5)

#define ADC_CLK				3000000

#define MAX_12BIT                       ((1 << 12) - 1)

//#define DEBUG_BAT_ADC printk
#define DEBUG_BAT_ADC(...) 

int counter = 0;
static int bat_capacity = 0;
static int battery_level = 0;
struct tscadc {
	int			irq;
	void __iomem		*tsc_base;
	unsigned int		ctrl;
    struct timer_list bat_adc_timer;
    struct work_struct bat_adc_work;
    struct workqueue_struct *bat_adc_wq;
};

static unsigned int tscadc_readl(struct tscadc *ts, unsigned int reg)
{
	return readl(ts->tsc_base + reg);
}

static void tscadc_writel(struct tscadc *tsc, unsigned int reg,
					unsigned int val)
{
	writel(val, tsc->tsc_base + reg);
}
static void battery_adc_work(struct work_struct *work)
{
    unsigned int                    status = 0, irqclr=0, readx1=0;
    int                             i, fifo0count=0;
    int                             total_value = 0;
    int                             battery_adc_value = 0;
    int                             adc_voltage = 0;

    struct tscadc *ts_dev =
        container_of(work, struct tscadc, bat_adc_work);
    status = tscadc_readl(ts_dev, TSCADC_REG_IRQSTATUS);
    DEBUG_BAT_ADC("interrupt status is %x\n", status);
    if (status & TSCADC_IRQENB_FIFO0THRES)
    {
        fifo0count = tscadc_readl(ts_dev, TSCADC_REG_FIFO0CNT);
        if(!fifo0count)
            return;
        DEBUG_BAT_ADC("fifo0count is %d\n", fifo0count);
        total_value = 0;
        for (i = 0; i < (fifo0count); i++)
        {
            readx1 = 0;
            readx1 = tscadc_readl(ts_dev, TSCADC_REG_FIFO0);
            readx1 = readx1 & 0xfff;
            total_value += readx1;
           // DEBUG_BAT_ADC(KERN_INFO "\n readx1 hex %x decimal %d \n", readx1, readx1);
        }
        battery_adc_value = total_value/fifo0count;
        adc_voltage = (battery_adc_value * 1800) >> 12;
        battery_level = (battery_adc_value * 4266) >> 12;
#if 1
        if(!gpio_get_value(DC_ADAPTER_STATUS))
        {
            /*AC is present  */
            battery_level = battery_level - 200;
        }
#endif
	if(counter > 0)
	{
		if(battery_level <= 3300)
		{
			if(counter > 2)
			{
				bat_capacity = 0;
				goto BATTERY_ADC;
			}
			else
			{
				counter++;
				DEBUG_BAT_ADC("counter1 is %d\n",counter);

			}
		}
		else
		{
			counter = 0;
		}
	}
        if(battery_level > 3984)
             bat_capacity = 100;
        else if(battery_level > 3904)
            bat_capacity = 90;
        else if(battery_level > 3832)
            bat_capacity = 80;
        else if(battery_level > 3760)
            bat_capacity = 70;
        else if(battery_level > 3720)
            bat_capacity = 60;
        else if(battery_level > 3680)
            bat_capacity = 50;
        else if(battery_level > 3640)
            bat_capacity = 40;
        else if(battery_level > 3600)
            bat_capacity = 30;
        else if(battery_level > 3300)
            bat_capacity = 10;
        else if(battery_level > 3100)
          {
            counter++;
            DEBUG_BAT_ADC("counter is %d\n",counter);
          }
        else
            bat_capacity = 0;

BATTERY_ADC:
        DEBUG_BAT_ADC(KERN_INFO "\n adc value %d adc voltage %d battery_level %d bat_capacity is %d\n", battery_adc_value, adc_voltage, battery_level, bat_capacity);

        /*clear the interrupt */
        tscadc_writel(ts_dev, TSCADC_REG_IRQSTATUS, TSCADC_IRQENB_FIFO0THRES);
    }
}

void get_battery_capacity(int *battery_capacity)
{
    *battery_capacity = bat_capacity;
}
EXPORT_SYMBOL_GPL(get_battery_capacity);

void get_battery_voltage(int *battery_voltage)
{
    *battery_voltage = battery_level;
}
EXPORT_SYMBOL_GPL(get_battery_voltage);


static void tsc_step_config(struct tscadc *ts_dev)
{
	unsigned int	stepconfig = 0;
	unsigned int	delay;
	int i;

	/* Configure the Step registers */
	delay = TSCADC_STEPCONFIG_SAMPLEDLY | TSCADC_STEPCONFIG_OPENDLY;

	stepconfig = TSCADC_STEPCONFIG_2SAMPLES_AVG | TSCADC_STEPCONFIG_INP_5 | TSCADC_STEPCONFIG_INP_4 | TSCADC_STEPCONFIG_MODE_SWSYNC_CONT;

    for (i = 1; i <= 1; i++)
    {
		tscadc_writel(ts_dev, TSCADC_REG_STEPCONFIG(i), stepconfig);
		tscadc_writel(ts_dev, TSCADC_REG_STEPDELAY(i), delay);
    }
	
	tscadc_writel(ts_dev, TSCADC_REG_SE, TSCADC_STPENB_STEPENB);
}

static void tsc_idle_config(struct tscadc *ts_config)
{
	/* Idle mode touch screen config */
	unsigned int	 idleconfig;

    idleconfig = TSCADC_STEPCONFIG_INP_5 | TSCADC_STEPCONFIG_INP_4;

	tscadc_writel(ts_config, TSCADC_REG_IDLECONFIG, idleconfig);
}

/*
* The functions for inserting/removing driver as a module.
*/
static void battery_adc_timer(unsigned long data)
{
    struct tscadc *ts_dev = (struct tscadc *)data;
    int ret;

    /* schedule next call to state machine */
    mod_timer(&ts_dev->bat_adc_timer,
            jiffies + msecs_to_jiffies(1000));

#if 0
    ret = schedule_work(&ts_dev->bat_adc_work);
    if (!ret)
    {
        printk("ts_dev is %x %x\n", ts_dev, &ts_dev->bat_adc_work); 
        printk("battery adc: state machine failed to schedule\n");
    }
#else
    ret = queue_work(ts_dev->bat_adc_wq, &ts_dev->bat_adc_work);
    if(!ret)
        printk("battery adc: failed to queue the work\n");
#endif

}

static	int __devinit tscadc_probe(struct platform_device *pdev)
{
	struct tscadc			*ts_dev;
	int				err;
	int				clk_value;
	int				clock_rate, irqenable, ctrl;
	struct resource			*res;
	struct clk			*clk;

	DEBUG_BAT_ADC("tsc probe entered 1 \n");
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);				/*get resources*/
	if (!res) {
		dev_err(&pdev->dev, "no memory resource defined.\n");
		return -EINVAL;
	}

	/* Allocate memory for device */
	ts_dev = kzalloc(sizeof(struct tscadc), GFP_KERNEL);				/*allocate memory and memset it*/	
	if (!ts_dev) {
		dev_err(&pdev->dev, "failed to allocate memory.\n");
		return -ENOMEM;
	}

	res =  request_mem_region(res->start, resource_size(res), pdev->name);		/*request for I/O memory map region*/
	if (!res) {
		dev_err(&pdev->dev, "failed to reserve registers.\n");
		err = -EBUSY;
		goto err_free_mem;
	}

	ts_dev->tsc_base = ioremap(res->start, resource_size(res));			/*map the two memories*/
	if (!ts_dev->tsc_base) {
		dev_err(&pdev->dev, "failed to map registers.\n");
		err = -ENOMEM;
		goto err_release_mem;
	}

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

    ts_dev->bat_adc_wq = create_singlethread_workqueue("bat_adc_wq");
	if (!(ts_dev->bat_adc_wq)) {
		printk(KERN_ALERT "bat_adc_wq: create workqueue failed\n");
		return -ENOMEM;		
	}

    init_timer(&ts_dev->bat_adc_timer);
    ts_dev->bat_adc_timer.data = (unsigned long)ts_dev;
    ts_dev->bat_adc_timer.function = battery_adc_timer;
    INIT_WORK(&ts_dev->bat_adc_work, battery_adc_work);

    printk("ts_dev is %x %x\n", ts_dev, &ts_dev->bat_adc_work); 
	clk = clk_get(&pdev->dev, "adc_tsc_fck");					/*get clock*/
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "failed to get TSC fck\n");
		err = PTR_ERR(clk);
		goto err_free_irq;
	}
	clock_rate = clk_get_rate(clk);
	clk_value = clock_rate / ADC_CLK;
	if (clk_value < 7) {
		dev_err(&pdev->dev, "clock input less than min clock requirement\n");
		err = -EINVAL;
		goto err_fail;
	}
	/* TSCADC_CLKDIV needs to be configured to the value minus 1 */
	clk_value = clk_value - 1;
	tscadc_writel(ts_dev, TSCADC_REG_CLKDIV, clk_value);				

	/* Set the control register bits */
	ctrl = TSCADC_CNTRLREG_STEPCONFIGWRT |
			TSCADC_CNTRLREG_STEPID;

	tscadc_writel(ts_dev, TSCADC_REG_CTRL, ctrl);
	ts_dev->ctrl = ctrl;

	/* Set register bits for Idel Config Mode */
	tsc_idle_config(ts_dev);

	/* IRQ Enable */
	irqenable = TSCADC_IRQENB_FIFO0THRES;

	tscadc_writel(ts_dev, TSCADC_REG_IRQENABLE, irqenable);

	tsc_step_config(ts_dev);

	tscadc_writel(ts_dev, TSCADC_REG_FIFO0THR, 6);

	ctrl |= TSCADC_CNTRLREG_TSCSSENB;
	tscadc_writel(ts_dev, TSCADC_REG_CTRL, ctrl);

	device_init_wakeup(&pdev->dev, true);

	platform_set_drvdata(pdev, ts_dev);
    mod_timer(&ts_dev->bat_adc_timer, jiffies + 1);
	return 0;

err_fail:
	pm_runtime_disable(&pdev->dev);
err_free_irq:
err_unmap_regs:
	iounmap(ts_dev->tsc_base);
err_release_mem:
	release_mem_region(res->start, resource_size(res));
err_free_mem:
	kfree(ts_dev);
	return err;
}

static int __devexit tscadc_remove(struct platform_device *pdev)
{
	struct tscadc		*ts_dev = platform_get_drvdata(pdev);
	struct resource		*res;
    printk("%s %d \n", __func__, __LINE__);

    /* cancel state machine timer */
    tscadc_writel(ts_dev, TSCADC_REG_IRQSTATUS, TSCADC_IRQENB_FIFO0THRES);
    del_timer_sync(&ts_dev->bat_adc_timer);
    cancel_work_sync(&ts_dev->bat_adc_work);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(ts_dev->tsc_base);
	release_mem_region(res->start, resource_size(res));

	pm_runtime_disable(&pdev->dev);

	kfree(ts_dev);

	device_init_wakeup(&pdev->dev, 0);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static int tscadc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tscadc *ts_dev = platform_get_drvdata(pdev);
	unsigned int idle;
    printk("%s %d \n", __func__, __LINE__);
    del_timer_sync(&ts_dev->bat_adc_timer);
    cancel_work_sync(&ts_dev->bat_adc_work);
	if (device_may_wakeup(&pdev->dev)) {
		idle = tscadc_readl(ts_dev, TSCADC_REG_IRQENABLE);
		tscadc_writel(ts_dev, TSCADC_REG_IRQENABLE,
				(idle | TSCADC_IRQENB_HW_PEN));
		tscadc_writel(ts_dev, TSCADC_REG_IRQWAKEUP, TSCADC_IRQWKUP_ENB);
	}

	/* module disable */
	idle = 0;
	idle = tscadc_readl(ts_dev, TSCADC_REG_CTRL);
	idle &= ~(TSCADC_CNTRLREG_TSCSSENB);
	tscadc_writel(ts_dev, TSCADC_REG_CTRL, idle);

	pm_runtime_put_sync(&pdev->dev);

	return 0;

}

static int tscadc_resume(struct platform_device *pdev)
{
	struct tscadc *ts_dev = platform_get_drvdata(pdev);
	unsigned int restore;

	pm_runtime_get_sync(&pdev->dev);

	if (device_may_wakeup(&pdev->dev)) {
		tscadc_writel(ts_dev, TSCADC_REG_IRQWAKEUP,
				TSCADC_IRQWKUP_DISABLE);
		tscadc_writel(ts_dev, TSCADC_REG_IRQCLR, TSCADC_IRQENB_HW_PEN);
	}

	/* context restore */
	tscadc_writel(ts_dev, TSCADC_REG_CTRL, ts_dev->ctrl);
	tsc_idle_config(ts_dev);
	tsc_step_config(ts_dev);
	tscadc_writel(ts_dev, TSCADC_REG_FIFO1THR, 6);
	restore = tscadc_readl(ts_dev, TSCADC_REG_CTRL);
	tscadc_writel(ts_dev, TSCADC_REG_CTRL,
			(restore | TSCADC_CNTRLREG_TSCSSENB));
    mod_timer(&ts_dev->bat_adc_timer, jiffies + 1);
	return 0;
}

static struct platform_driver ti_tsc_driver = {
	.probe	  = tscadc_probe,
	.remove	 = __devexit_p(tscadc_remove),
	.shutdown	 = __devexit_p(tscadc_remove),
	.driver	 = {
		.name   = "tsc",
		.owner  = THIS_MODULE,
	},
	.suspend = tscadc_suspend,
	.resume  = tscadc_resume,
};

static int __init ti_tsc_init(void)
{
	return platform_driver_register(&ti_tsc_driver);
}
module_init(ti_tsc_init);

static void __exit ti_tsc_exit(void)
{
	platform_driver_unregister(&ti_tsc_driver);
}
module_exit(ti_tsc_exit);

MODULE_DESCRIPTION("TI touchscreen controller driver");
MODULE_AUTHOR("Rachna Patil <rachna@ti.com>");
MODULE_LICENSE("GPL");
