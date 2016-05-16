/*
 * mst3000_charger.h
 *
 *  Created on: May 16, 2016
 *      Author: erick
 */

#ifndef DRIVERS_POWER_MST3000_CHARGER_H_
#define DRIVERS_POWER_MST3000_CHARGER_H_

// Platform data for the MST3000 charger driver

struct mst3000_charger_pdata {
	int pg_pin; // Power good pin - must be IRQ capable
	int ce_pin; // Charger enable pin - output to enable charging
};


#endif /* DRIVERS_POWER_MST3000_CHARGER_H_ */
