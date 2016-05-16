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
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <mach/board-am335xevm.h>
#include <asm/unaligned.h>
#include <mst3000_charger.h>


#define CHARGER_DEBUG printk
//#define CHARGER_DEBUG(...)

#define REG_ISC			0x00			// Input source control
#define REG_POC			0x01			// Power-On configuration
#define REG_CCC			0x02			// Charge current control
#define REG_PTCC		0x03			// Pre-charge/Termination current control
#define REG_CVC			0x04			// Charge voltage control
#define REG_CTTCR		0x05			// Charge termination/timer control
#define REG_TRC			0x06			// Thermal regulation control
#define REG_MOC			0x07			// Misc operation control
#define REG_SS			0x08			// System status
#define REG_F			0x09			// Fault register
#define REG_VPRS		0x0A			// Vendor/part/revision status




static int ac_present_flag = 0;
struct mst3000_charger_data {
	uint32_t reg_base;
	int irq;
	spinlock_t lock;

	struct power_supply battery;
	struct power_supply ac;
	struct timer_list sm_timer;
	int battery_capacity;
	struct i2c_client *client;
};


static struct mst3000_charger_pdata *pdata;

/* temporary variable used between allgo_etab_battery_probe() and allgo_etab_battery_open() */
static struct mst3000_charger_data *battery_data;


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


static int mst3000_charger_read_i2c(struct i2c_client *client, u8 reg, bool single)
{
	struct i2c_msg msg[2];
	unsigned char data[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	if (single)
		msg[1].len = 1;
	else
		msg[1].len = 2;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	if (!single)
		ret = get_unaligned_le16(data);
	else
		ret = data[0];

	return ret;
}

static int mst3000_charger_write_i2c(struct i2c_client *client, u8 reg, int value, bool single)
{
	struct i2c_msg msg;
	unsigned char data[4];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	data[0] = reg;
	if (single) {
		data[1] = (unsigned char)value;
		msg.len = 2;
	} else {
		put_unaligned_le16(value, &data[1]);
		msg.len = 3;
	}

	msg.buf = data;
	msg.addr = client->addr;
	msg.flags = 0;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static void mst3000_charger_update_boost_control(struct i2c_client *client, int online)
{
	int value = online?0x11:0x31;
	mst3000_charger_write_i2c(client, REG_POC, value, true);
}


static int mst3000_charger_ac_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct mst3000_charger_data *data = container_of(psy,
			struct mst3000_charger_data, ac);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		/*Read the status of gpio pin for power suppply status */
		val->intval = !(gpio_get_value(pdata->pg_pin));
		mst3000_charger_update_boost_control(data->client, val->intval);
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
	struct mst3000_charger_data *data = container_of(psy,
			struct mst3000_charger_data, battery);
	int ret = 0;
	int charge_status = 0, battery_capacity = 0,battery_charge_status = 0;
	int battery_voltage = 0;
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		/*Read the status of gpio pin for battery charging status  */
		if(!(gpio_get_value(pdata->pg_pin))){
			charge_status = !(gpio_get_value(pdata->ce_pin));
			CHARGER_DEBUG("read battery charging status %d\n", charge_status);
			if(charge_status == 1){
				/* Battery charging */
				val->intval = 1;
			} 
			else{
				/* Battery Full*/
				val->intval= 4;
			}
			mst3000_charger_update_boost_control(data->client, 1);
		}
		else 
		{
			/* Battery discharging */
			val->intval=2;
			mst3000_charger_update_boost_control(data->client, 0);
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
	struct mst3000_charger_data *data = dev_id;
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
	struct mst3000_charger_data *battery_data = (struct allgo_etab_battery_data *)data;
	int ret;
	int battery_capacity = 0;

	/* schedule next call to state machine */
	mod_timer(&battery_data->sm_timer,
			jiffies + msecs_to_jiffies(30000));
	battery_data->battery_capacity = battery_capacity;
	power_supply_changed(&battery_data->battery);
}

static int mst3000_charger_probe(struct i2c_client *client,
		 const struct i2c_device_id *id)
{
	int ret;
	struct resource *r;
	struct mst3000_charger_data *data;
	
	pdata = client->dev.platform_data;
	
	CHARGER_DEBUG("\n\n %s - entered \n\n", __func__);
	
	if (pdata == NULL) 
	{
		CHARGER_DEBUG("\n\n ERROR: No platform data \n\n");
		return -EINVAL;
	}
	
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
	data->ac.get_property = mst3000_charger_ac_get_property;
	data->ac.name = "ac";
	data->ac.type = POWER_SUPPLY_TYPE_MAINS;
	data->client = client;
	init_timer(&data->sm_timer);
	data->sm_timer.data = (unsigned long)data;
	data->sm_timer.function = battery_state_timer;


	/* Request the corresponding gpio pin before reading the status */
	gpio_request(pdata->pg_pin, "dc_adapter_status");
	gpio_direction_input(pdata->pg_pin);
	gpio_request(pdata->ce_pin, "charge_status");
	gpio_direction_input(pdata->ce_pin);
	/* Read the status of gpio pin to check the dc power status */
	ac_present_flag = !(gpio_get_value(pdata->pg_pin));

	if(ac_present_flag)
	{
		/* Wait for the interrupt i.e DC jack removed */
		ret = request_irq(gpio_to_irq(pdata->pg_pin), dc_jack_interrupt,  IRQF_DISABLED | IRQ_TYPE_LEVEL_HIGH, dev_name(&client->dev), data);
		if (ret)
			goto err_request_irq_failed;
	}
	else
	{
		/* Wait for the inerrupt i.e DC jack inserted */
		ret = request_irq(gpio_to_irq(pdata->pg_pin), dc_jack_interrupt,  IRQF_DISABLED | IRQ_TYPE_LEVEL_LOW, dev_name(&client->dev), data);
		if (ret)
			goto err_request_irq_failed;	
	}
	ret = power_supply_register(&client->dev, &data->ac);
	if (ret)
		goto err_ac_failed;

	ret = power_supply_register(&client->dev, &data->battery);
	if (ret)
		goto err_battery_failed;

	i2c_set_clientdata(client, data);
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

static int mst3000_charger_remove(struct platform_device *pdev)
{
	struct mst3000_charger_data *data = platform_get_drvdata(pdev);
	del_timer_sync(&data->sm_timer);

	power_supply_unregister(&data->battery);
	power_supply_unregister(&data->ac);

	free_irq(data->irq, data);
	kfree(data);
	battery_data = NULL;
	return 0;
}


static const struct i2c_device_id mst3000_id[] = {
	{ "bq24196", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, mst3000_id);

static struct i2c_driver mst3000_charger_driver = {
	.driver = {
		.name = "mst3000_charger",
	},
	.probe = mst3000_charger_probe,
	.remove = mst3000_charger_remove,
	.id_table = mst3000_id,
};


static inline int __init mst3000_charger_i2c_init(void)
{
	int ret = i2c_add_driver(&mst3000_charger_driver);
	if (ret)
		printk(KERN_ERR "Unable to register MST3000 charger i2c driver\n");
	else
		printk("Registered MST3000 charger driver\n");

	return ret;
}

static inline void __exit mst3000_charger_i2c_exit(void)
{
	i2c_del_driver(&mst3000_charger_driver);

}

module_init(mst3000_charger_i2c_init);
module_exit(mst3000_charger_i2c_exit);

MODULE_AUTHOR("Erick Reyes erick@toolplanet.co.jp");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BQ24196 charger driver for the MST3000");

