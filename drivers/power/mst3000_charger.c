/* drivers/power/allgo_etab_battery.c
 *
 * Power supply driver for the allgo_etab emulator
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <mach/board-am335xevm.h>

//#define CHARGER_DEBUG printk
#define CHARGER_DEBUG(...)


static int ac_present_flag = 0;
struct allgo_etab_battery_data {
	uint32_t reg_base;
	int irq;
	spinlock_t lock;

	struct power_supply battery;
	struct power_supply ac;
    struct timer_list sm_timer;
    int battery_capacity;
};

#ifndef GPIO_TO_PIN
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))
#endif

#define DC_ADAPTER_STATUS 	GPIO_TO_PIN(2, 1)
#define CHARGE_STATUS 		GPIO_TO_PIN(1, 27)


/* temporary variable used between allgo_etab_battery_probe() and allgo_etab_battery_open() */
static struct allgo_etab_battery_data *battery_data;


enum {
	/* status register */
	BATTERY_INT_STATUS	    = 0x00,
	/* set this to enable IRQ */
	BATTERY_INT_ENABLE	    = 0x04,

	BATTERY_AC_ONLINE       = 0x08,
	BATTERY_STATUS          = 0x0C,
	BATTERY_HEALTH          = 0x10,
	BATTERY_PRESENT         = 0x14,
	BATTERY_CAPACITY        = 0x18,

	BATTERY_STATUS_CHANGED	= 1U << 0,
	AC_STATUS_CHANGED   	= 1U << 1,
	BATTERY_INT_MASK        = BATTERY_STATUS_CHANGED | AC_STATUS_CHANGED,
};


static int allgo_etab_ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct allgo_etab_battery_data *data = container_of(psy,
		struct allgo_etab_battery_data, ac);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
              /*Read the status of gpio pin for power suppply status */
		val->intval = !(gpio_get_value(DC_ADAPTER_STATUS));
                CHARGER_DEBUG("read power supply status %d\n", val->intval);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int allgo_etab_battery_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct allgo_etab_battery_data *data = container_of(psy,
		struct allgo_etab_battery_data, battery);
	int ret = 0;
        int charge_status = 0, battery_capacity = 0,battery_charge_status = 0;
	int battery_voltage = 0;
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		/*Read the status of gpio pin for battery charging status  */
		if(!(gpio_get_value(DC_ADAPTER_STATUS))){
			charge_status = !(gpio_get_value(CHARGE_STATUS));
			CHARGER_DEBUG("read battery charging status %d\n", charge_status);
			if(charge_status == 1){
				/* Battery charging */
				val->intval = 1;
			} 
			else{
				/* Battery Full*/
				val->intval= 4;
			}
		}
		else 
		{
			/* Battery discharging */
			val->intval=2;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property allgo_etab_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
};

static enum power_supply_property allgo_etab_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

/* DC jack interrupt handler (handler for dc jack insert and remove)*/
static irqreturn_t dc_jack_interrupt(int irq, void *dev_id)
{
	unsigned long irq_flags;
	struct allgo_etab_battery_data *data = dev_id;
	spin_lock_irqsave(&data->lock, irq_flags);
	if(ac_present_flag)
	{
                CHARGER_DEBUG("DC_JACK is removed\n");
		ac_present_flag = 0;
		irq_set_irq_type(irq, IRQ_TYPE_LEVEL_LOW);	
	}
	else
	{
                CHARGER_DEBUG("DC_JACK is inserted\n");
		ac_present_flag = 1;
		irq_set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH);	
	}
	power_supply_changed(&data->battery);
	power_supply_changed(&data->ac);
	spin_unlock_irqrestore(&data->lock, irq_flags);
        return IRQ_HANDLED;
}

/*
* The functions for inserting/removing driver as a module.
*/
static void battery_state_timer(unsigned long data)
{
    struct allgo_etab_battery_data *battery_data = (struct allgo_etab_battery_data *)data;
    int ret;
    int battery_capacity = 0;

    /* schedule next call to state machine */
    mod_timer(&battery_data->sm_timer,
            jiffies + msecs_to_jiffies(30000));
    battery_data->battery_capacity = battery_capacity;
    power_supply_changed(&battery_data->battery);
}

static int allgo_etab_battery_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *r;
	struct allgo_etab_battery_data *data;

        CHARGER_DEBUG("\n\n %s - entered \n\n", __func__);
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}
	spin_lock_init(&data->lock);

	data->battery.properties = allgo_etab_battery_props;
	data->battery.num_properties = ARRAY_SIZE(allgo_etab_battery_props);
	data->battery.get_property = allgo_etab_battery_get_property;
	data->battery.name = "battery";
	data->battery.type = POWER_SUPPLY_TYPE_BATTERY;

	data->ac.properties = allgo_etab_ac_props;
	data->ac.num_properties = ARRAY_SIZE(allgo_etab_ac_props);
	data->ac.get_property = allgo_etab_ac_get_property;
	data->ac.name = "ac";
	data->ac.type = POWER_SUPPLY_TYPE_MAINS;

    init_timer(&data->sm_timer);
    data->sm_timer.data = (unsigned long)data;
    data->sm_timer.function = battery_state_timer;


        /* Request the corresponding gpio pin before reading the status */
	gpio_request(DC_ADAPTER_STATUS, "dc_adapter_status");
	gpio_direction_input(DC_ADAPTER_STATUS);
	gpio_request(CHARGE_STATUS, "charge_status");
	gpio_direction_input(CHARGE_STATUS);
        /* Read the status of gpio pin to check the dc power status */
	ac_present_flag = !(gpio_get_value(DC_ADAPTER_STATUS));

	if(ac_present_flag)
	{
                /* Wait for the interrupt i.e DC jack removed */
		ret = request_irq(gpio_to_irq(DC_ADAPTER_STATUS), dc_jack_interrupt,  IRQF_DISABLED | IRQ_TYPE_LEVEL_HIGH, pdev->name, data);
		if (ret)
			goto err_request_irq_failed;
	}
	else
	{
                /* Wait for the inerrupt i.e DC jack inserted */
		ret = request_irq(gpio_to_irq(DC_ADAPTER_STATUS), dc_jack_interrupt,  IRQF_DISABLED | IRQ_TYPE_LEVEL_LOW, pdev->name, data);
		if (ret)
			goto err_request_irq_failed;	
	}
	ret = power_supply_register(&pdev->dev, &data->ac);
	if (ret)
		goto err_ac_failed;

	ret = power_supply_register(&pdev->dev, &data->battery);
	if (ret)
		goto err_battery_failed;

	platform_set_drvdata(pdev, data);
	battery_data = data;

	mod_timer(&data->sm_timer, jiffies + 1);

	CHARGER_DEBUG("\n\n %s - exited without error \n\n", __func__);
	return 0;

err_battery_failed:
	power_supply_unregister(&data->ac);
err_ac_failed:
	free_irq(data->irq, data);
err_request_irq_failed:
err_no_irq:
err_no_io_base:
	kfree(data);
err_data_alloc_failed:
        CHARGER_DEBUG("\n\n %s - exited with error \n\n", __func__);
	return ret;
}

static int allgo_etab_battery_remove(struct platform_device *pdev)
{
	struct allgo_etab_battery_data *data = platform_get_drvdata(pdev);
    del_timer_sync(&data->sm_timer);

	power_supply_unregister(&data->battery);
	power_supply_unregister(&data->ac);

	free_irq(data->irq, data);
	kfree(data);
	battery_data = NULL;
	return 0;
}

static int allgo_etab_battery_suspend(struct platform_device *pdev, pm_message_t state)
{
    printk("%s %d \n", __func__, __LINE__);
	return 0;
}

static int allgo_etab_battery_resume(struct platform_device *pdev)
{
    printk("%s %d \n", __func__, __LINE__);
	return 0;
}

static struct platform_driver allgo_etab_battery_device = {
	.probe		= allgo_etab_battery_probe,
	.remove		= allgo_etab_battery_remove,
    .suspend = allgo_etab_battery_suspend,
	.resume  = allgo_etab_battery_resume,
	.driver = {
		.name = "mst3000_charger"
	}
};

static int __init allgo_etab_battery_init(void)
{
	return platform_driver_register(&allgo_etab_battery_device);
}

static void __exit allgo_etab_battery_exit(void)
{
	platform_driver_unregister(&allgo_etab_battery_device);
}

module_init(allgo_etab_battery_init);
module_exit(allgo_etab_battery_exit);

MODULE_AUTHOR("Mike Lockwood lockwood@android.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Battery driver for the Goldfish emulator");

