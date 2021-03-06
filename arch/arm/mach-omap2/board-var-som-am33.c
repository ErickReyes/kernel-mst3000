/*
 * Code for Variscite AM335X SOM.
 *
 * Copyright (C) 2012 Variscite, Ltd. - http://www.variscite.com/
 *
 * Copyright (C) 2011 Texas Instruments, Inc. - http://www.ti.com/
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/micrel_phy.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/platform_data/ti_adc.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/wl12xx.h>
#include <linux/ethtool.h>
#include <linux/mfd/tps65910.h>
#include <linux/pwm_backlight.h>
#include <linux/reboot.h>
#include <linux/pwm/pwm.h>
#include <linux/ti_wilink_st.h>
#include <linux/input/ti_tsc.h>
#include <linux/mfd/ti_tscadc.h>

#include <video/da8xx-fb.h>
#include <mach/hardware.h>
#include <mach/board-am335xevm.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/asp.h>

#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/lcdc.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/emif.h>
#include <plat/nand.h>
#include <plat/omap-pm.h>
#include <plat/omap_device.h>

#include "common.h"
#include "board-flash.h"
#include "cpuidle33xx.h"
#include "mux.h"
#include "devices.h"
#include "hsmmc.h"

#include <mst3000_charger.h>

#define VAR_LCD_UTM     0
#define VAR_LCD_CTW6120 1

#define PANEL_POWER_DELAY	100



/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))


#define LCD_DVDD_EN 	GPIO_TO_PIN(1, 17)
#define LCD_EN 			GPIO_TO_PIN(1, 24)
#define LCD_AVDD_EN 	GPIO_TO_PIN(1, 22)
#define LCD_VGH_EN 		GPIO_TO_PIN(1, 19)
#define LCD_VGL_EN		GPIO_TO_PIN(1, 25)
#define LCD_RESET		GPIO_TO_PIN(1, 23)

#define TOUCH_RESET		GPIO_TO_PIN(1, 18)

#define CHG_EN			GPIO_TO_PIN(1, 26)

#define PWR_CNTRL		GPIO_TO_PIN(3, 7)


#define NO_OF_MAC_ADDR		3
static char am335x_mac_addr[NO_OF_MAC_ADDR][ETH_ALEN];

/* LCD */
static const struct display_panel disp_panel = {
	WVGA,
	32,
	32,
	COLOR_ACTIVE,
};

static struct lcd_ctrl_config lcd_cfg = {
	&disp_panel,
	.ac_bias		= 255,
	.ac_bias_intrpt		= 0,
	.dma_burst_sz		= 16,
	.bpp			= 32,
	.fdd			= 0x80,
	.tft_alt_mode		= 0,
	.stn_565_mode		= 0,
	.mono_8bit_mode		= 0,
	.invert_line_clock	= 1,
	.invert_frm_clock	= 1,
	.sync_edge		= 0,
	.sync_ctrl		= 1,
	.raster_order		= 0,
};

struct da8xx_lcdc_platform_data VAR_LCD_pdata = {
	.manu_name		= "Variscite",
	.controller_data	= &lcd_cfg,
	.type			= "VAR-WVGA",
};

void yaoyu_panel_power_ctrl (int state)
{
	printk("yaoyu_panel_power_ctrl state %d\n", state);
	if (state)
	{
		/*LCD Power on Sequence */
		gpio_request(LCD_DVDD_EN, "lcd_dvdd_en");
		gpio_direction_output(LCD_DVDD_EN, 1);
		mdelay(PANEL_POWER_DELAY);

		gpio_request(LCD_EN, "lcd_en");
		gpio_direction_output(LCD_EN, 1);
		mdelay(PANEL_POWER_DELAY);

		gpio_request(LCD_AVDD_EN, "lcd_avdd_en");
		gpio_direction_output(LCD_AVDD_EN, 1);
		mdelay(PANEL_POWER_DELAY);

		gpio_request(LCD_VGL_EN, "lcd_vgl_en");
		gpio_direction_output(LCD_VGL_EN, 1);
		mdelay(PANEL_POWER_DELAY);

		gpio_request(LCD_VGH_EN, "lcd_vgh_en");
		gpio_direction_output(LCD_VGH_EN, 1);
		mdelay(PANEL_POWER_DELAY);

		gpio_request(LCD_RESET, "lcd_reset");
		gpio_direction_output(LCD_RESET, 1);

	}
	else
	{
		/*LCD Power Off Sequence */

		gpio_request(LCD_RESET, "lcd_reset");
		gpio_direction_output(LCD_RESET, 0);

		gpio_request(LCD_VGH_EN, "lcd_vgh_en");
		gpio_direction_output(LCD_VGH_EN, 0);


		gpio_request(LCD_VGL_EN, "lcd_vgl_en");
		gpio_direction_output(LCD_VGL_EN, 0);


		gpio_request(LCD_AVDD_EN, "lcd_avdd_en");
		gpio_direction_output(LCD_AVDD_EN, 0);


		gpio_request(LCD_EN, "lcd_en");
		gpio_direction_output(LCD_EN, 0);

		gpio_request(LCD_DVDD_EN, "lcd_dvdd_en");
		gpio_direction_output(LCD_DVDD_EN, 0);
	}
}

struct da8xx_lcdc_platform_data YAOYU_LCD_DATA = {
	.manu_name		= "Yaoyu",
	.controller_data	= &lcd_cfg,
	.type			= "YM700T-011A",
	.panel_power_ctrl = yaoyu_panel_power_ctrl,
};

/* Audio */
static u8 am335x_iis_serializer_direction1[] = {
	INACTIVE_MODE,	INACTIVE_MODE,	RX_MODE,	TX_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
};

static struct snd_platform_data var_am335x_som_snd_data0 = {
	.tx_dma_offset	= 0x46000000,	/* McASP0 */
	.rx_dma_offset	= 0x46000000,
	.op_mode	= DAVINCI_MCASP_IIS_MODE,
	.num_serializer	= ARRAY_SIZE(am335x_iis_serializer_direction1),
	.tdm_slots	= 2,
	.serial_dir	= am335x_iis_serializer_direction1,
	.asp_chan_q	= EVENTQ_2,
	.version	= MCASP_VERSION_3,
	.txnumevt	= 32,
	.rxnumevt	= 32,
	.get_context_loss_count	=
			omap_pm_get_dev_context_loss_count,
};

/* MMC */
static struct omap2_hsmmc_info am335x_mmc[] __initdata = {
	{
		.mmc            = 1,
		.caps           = MMC_CAP_4_BIT_DATA,
		.gpio_cd        = GPIO_TO_PIN(1, 28),
		.gpio_wp        = -EINVAL,
		.ocr_mask       = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3V3 */
	},
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{}      /* Terminator */
};


#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	AM33XX_MUX(I2C0_SDA, OMAP_MUX_MODE7 | AM33XX_INPUT_EN),
	AM33XX_MUX(I2C0_SCL, OMAP_MUX_MODE7 |AM33XX_INPUT_EN ),
	AM33XX_MUX(SPI0_SCLK, OMAP_MUX_MODE7 | AM33XX_SLEWCTRL_SLOW |
				AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(EMU1, OMAP_MUX_MODE7 |
				AM33XX_INPUT_EN | AM33XX_PULL_UP),

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define	board_mux	NULL
#endif

/* module pin mux structure */
struct pinmux_config {
	const char *string_name; /* signal name format */
	int val; /* Options for the mux register value */
};
/* SOM revision GPIO */
static struct pinmux_config am33_var_som_rev_pin_mux[] = {
	{"lcd_data7.gpio2_13",	OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLDOWN},
	{"lcd_data5.gpio2_11",	OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLDOWN},
	{"lcd_vsync.gpio2_22",	OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLDOWN},

	{NULL, 0},
};

/* UART0 */
static struct pinmux_config uart0_pin_mux[] = {
	{"uart0_rxd.uart0_rxd", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"uart0_txd.uart0_txd", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL},
	{NULL, 0},
};

/* UART3 */
static struct pinmux_config uart3_pin_mux[] = {
	{"ecap0_in_pwm0_out.uart3_txd", OMAP_MUX_MODE1 | AM33XX_PULL_ENBL},
	{"spi0_cs1.uart3_rxd", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

#ifdef CONFIG_ANDROID
static struct pinmux_config haptics_pin_mux[] = {
	{"spi0_sclk.ehrpwm0A", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};
#else
static struct pinmux_config gpio_backlight_pin_mux[] = {
	{"spi0_sclk.gpio0_2", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};
#endif

static struct pinmux_config d_can0_pin_mux[] = {
	{"mii1_txd2.d_can0_rx", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{"mii1_txd3.d_can0_tx", OMAP_MUX_MODE1 | AM33XX_PULL_ENBL},
	{NULL, 0},
};

/* Module pin mux for LCDC */
static struct pinmux_config lcdc_pin_mux[] = {
	{"lcd_data0.lcd_data0",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data1.lcd_data1",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data2.lcd_data2",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data3.lcd_data3",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data4.lcd_data4",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data5.lcd_data5",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data6.lcd_data6",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data7.lcd_data7",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data8.lcd_data8",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data9.lcd_data9",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data10.lcd_data10",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data11.lcd_data11",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data12.lcd_data12",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data13.lcd_data13",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data14.lcd_data14",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data15.lcd_data15",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_vsync.lcd_vsync",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_hsync.lcd_hsync",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_pclk.lcd_pclk",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_ac_bias_en.lcd_ac_bias_en", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},


	{"gpmc_a1.gpio1_17", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{"gpmc_a6.gpio1_22", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{"gpmc_a9.gpio1_25", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{"gpmc_a3.gpio1_19", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{"gpmc_a8.gpio1_24", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{"gpmc_a7.gpio1_23", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},

	/* Touch panel reset signal */
	{"gpmc_a2.gpio1_18", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},

	{NULL, 0},
};

/* Pin mux for nand flash module */
static struct pinmux_config nand_pin_mux[] = {
	{"gpmc_ad0.gpmc_ad0",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad1.gpmc_ad1",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad2.gpmc_ad2",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad3.gpmc_ad3",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad4.gpmc_ad4",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad5.gpmc_ad5",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad6.gpmc_ad6",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad7.gpmc_ad7",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wait0.gpmc_wait0", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wpn.gpmc_wpn",	  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn0.gpmc_csn0",	  OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_advn_ale.gpmc_advn_ale",  OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_oen_ren.gpmc_oen_ren",	 OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_wen.gpmc_wen",     OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_ben0_cle.gpmc_ben0_cle",	 OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{NULL, 0},
};

/* Module pin mux for rmii1 */
static struct pinmux_config rmii1_pin_mux[] = {
	{"mii1_crs.rmii1_crs_dv", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxerr.mii1_rxerr", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txen.mii1_txen", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_txd1.mii1_txd1", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_txd0.mii1_txd0", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_rxd1.mii1_rxd1", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd0.mii1_rxd0", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"rmii1_refclk.rmii1_refclk", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for rgmii2 */
static struct pinmux_config rgmii2_pin_mux[] = {
	{"gpmc_a0.rgmii2_tctl", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"gpmc_a1.rgmii2_rctl", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a2.rgmii2_td3", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"gpmc_a3.rgmii2_td2", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"gpmc_a4.rgmii2_td1", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"gpmc_a5.rgmii2_td0", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"gpmc_a6.rgmii2_tclk", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"gpmc_a7.rgmii2_rclk", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a8.rgmii2_rd3", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a9.rgmii2_rd2", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a10.rgmii2_rd1", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a11.rgmii2_rd0", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_col.rmii2_refclk", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for rgmii2 strapping phase  */
static struct pinmux_config rgmii2_strapping_pin_mux[] = {
	{"gpmc_a7.gpio1_23", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{"gpmc_a8.gpio1_24", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{"gpmc_a9.gpio1_25", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{"gpmc_a10.gpio1_26", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{"gpmc_a11.gpio1_27", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{"gpmc_a1.gpio1_17", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

/* I2C1 */
static struct pinmux_config i2c1_pin_mux[] = {
	{"spi0_d1.i2c1_sda",    OMAP_MUX_MODE2 | AM33XX_SLEWCTRL_SLOW |
					AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
	{"spi0_cs0.i2c1_scl",   OMAP_MUX_MODE2 | AM33XX_SLEWCTRL_SLOW |
					AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
	{NULL, 0},
};

/* I2C0 */
static struct pinmux_config i2c0_pin_mux[] = {
	{"i2c0_sda.i2c0_sda",    OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT},
	{"i2c0_scl.i2c0_scl",   OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* Module pin mux for mcasp0 */
static struct pinmux_config mcasp0_pin_mux[] = {
	{"mcasp0_aclkx.mcasp0_aclkx", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mcasp0_fsx.mcasp0_fsx", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mcasp0_aclkr.mcasp0_axr2", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mcasp0_fsr.mcasp0_axr3", OMAP_MUX_MODE2 |	AM33XX_PIN_INPUT_PULLDOWN},
	{NULL, 0},
};

/* Module pin mux for mmc0 */
static struct pinmux_config mmc0_pin_mux[] = {
	{"mmc0_dat3.mmc0_dat3",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat2.mmc0_dat2",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat1.mmc0_dat1",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat0.mmc0_dat0",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_clk.mmc0_clk",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_cmd.mmc0_cmd",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	/*{"mcasp0_aclkr.mmc0_sdwp", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},*/
	{"gpmc_ben1.gpio1_28",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for battery charging and monitoring */

static struct pinmux_config mst3000_battery_pinmux[] = {
		{"gpmc_a11.gpio1_27", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP}, // Charging status
		{"gpmc_clk.gpio2_1", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP}, // AC power status
		{NULL, 0},
};

/* Module pin mux for MST3000 sub-cpu */
static struct pinmux_config mst3000_subcpu_pinmux[] = {
		{"gpmc_a5.gpio1_21", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP}, // Sub-CPU reset signal
		{"gpmc_a0.gpio1_16", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP}, // Sub-CPU ROM ISP signal
		{"mcasp0_ahclkr.gpio3_17", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP}, // Sub-CPU custom ISP signal
		{NULL, 0},
};


/*
* @pin_mux - single module pin-mux structure which defines pin-mux
*			details for all its pins.
*/
static void setup_pin_mux(struct pinmux_config *pin_mux)
{
	int i;

	for (i = 0; pin_mux->string_name != NULL; pin_mux++)
		omap_mux_init_signal(pin_mux->string_name, pin_mux->val);
}

#define AM33_VAR_SOM_REV_BIT0_GPIO GPIO_TO_PIN(2, 13)
#define AM33_VAR_SOM_REV_BIT1_GPIO GPIO_TO_PIN(2, 22)
#define AM33_VAR_SOM_REV_BIT2_GPIO GPIO_TO_PIN(2, 11)

static struct gpio som_rev_gpios[] __initdata = {
	{AM33_VAR_SOM_REV_BIT0_GPIO, GPIOF_IN,	"som_rev_bit_0"},
	{AM33_VAR_SOM_REV_BIT1_GPIO, GPIOF_IN,	"som_rev_bit_1"},
	{AM33_VAR_SOM_REV_BIT2_GPIO, GPIOF_IN,	"som_rev_bit_2"},

};


/************************ MST3000 **********************************/


static struct pinmux_config mst3000_keys_pin_mux[] = {
    // Power Key
	{"spi0_d0.gpio0_3", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	// Power control signal
	{"emu0.gpio3_7", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};


/* Configure GPIOs for power Key */
static struct gpio_keys_button mst3000_keys_gpio_buttons[] = {
    {
		.code                   = KEY_POWER,
		.gpio                   = GPIO_TO_PIN(0, 3),
		.active_low             = true,
		.desc                   = "power",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
};

static struct gpio_keys_platform_data mst3000_gpio_keys_info = {
	.buttons        = mst3000_keys_gpio_buttons,
	.nbuttons       = ARRAY_SIZE(mst3000_keys_gpio_buttons),
};

static struct platform_device mst3000_keys = {
	.name   = "gpio-keys",
	.id     = -1,
	.dev    = {
		.platform_data  = &mst3000_gpio_keys_info,
	},
};

static void mst3000_keys_init(int evm_id, int profile)
{
	int err;

	setup_pin_mux(mst3000_keys_pin_mux);
	err = platform_device_register(&mst3000_keys);
	if (err)
		pr_err("failed to register power key device\n");
}

//static struct platform_device mst3000_battery_charger = {
//        .name   = "mst3000_charger",
//        .id     = -1,
//};
//
//static void mst3000_battery_charger_init(int evm_id, int profile)
//{
//       int err;
//
//       err = platform_device_register(&mst3000_battery_charger);
//       if (err)
//               pr_err("failed to register battery charger\n");
//}



/* battery ADC */
static struct adc_data am335x_battey_adc_data  = {
	.adc_channels  = 4,
};

static struct mfd_tscadc_board battery_tscadc = {
	.adc_init = &am335x_battey_adc_data,
};


int get_var_am33_som_rev(void)
{
	static int som_rev = -1;
	int subversion;

	if (som_rev == (-1)) {
		int status;

		setup_pin_mux(am33_var_som_rev_pin_mux);

		status = gpio_request_array(som_rev_gpios,
				ARRAY_SIZE(som_rev_gpios));
		if (status) {
			pr_err("Error requesting som rev gpio: %d\n", status);
		}
		subversion = gpio_get_value(AM33_VAR_SOM_REV_BIT2_GPIO) ? 0 : 1;
		som_rev = (gpio_get_value(AM33_VAR_SOM_REV_BIT0_GPIO) |
				(gpio_get_value(AM33_VAR_SOM_REV_BIT1_GPIO) << 1)) +
			subversion;

		gpio_free_array(som_rev_gpios,
				ARRAY_SIZE(som_rev_gpios));

	}

	return som_rev;
}

const char *get_var_am33_som_rev_str(void)
{
	static char som_rev_str[32];

	sprintf(som_rev_str, "1.%d", get_var_am33_som_rev());

	return som_rev_str;
}

#define VAR_SOM_WLAN_PMENA_GPIO           GPIO_TO_PIN(3, 21)
#define VAR_SOM_WLAN_IRQ_GPIO             GPIO_TO_PIN(3, 20)
#define VAR_SOM_BT_PMENA_GPIO             GPIO_TO_PIN(3, 9)

static struct wl12xx_platform_data var_som_am33_wlan_data = {
	.irq = OMAP_GPIO_IRQ(VAR_SOM_WLAN_IRQ_GPIO),
	.wlan_enable_gpio = VAR_SOM_WLAN_PMENA_GPIO,
	.bt_enable_gpio = VAR_SOM_BT_PMENA_GPIO,
	.board_ref_clock = WL12XX_REFCLOCK_26, /* 26Mhz */
#ifdef CONFIG_MACH_AM335XEVM_WILINK8
        .board_tcxo_clock = WL12XX_TCXOCLOCK_26,
#endif
};

/* Module pin mux for wlan and bluetooth */
static struct pinmux_config mmc1_wl12xx_pin_mux[] = {
	{"gpmc_ad8.mmc1_dat0", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad9.mmc1_dat1", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad10.mmc1_dat2", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad11.mmc1_dat3", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn2.mmc1_cmd", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn1.mmc1_clk", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config uart1_wl12xx_pin_mux[] = {
	{"uart1_ctsn.uart1_ctsn", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"uart1_rtsn.uart1_rtsn", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"uart1_rxd.uart1_rxd", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"uart1_txd.uart1_txd", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL},
	{NULL, 0},
};

static struct pinmux_config wl12xx_pin_mux_var_som[] = {
	{"mcasp0_axr1.gpio3_20", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"mcasp0_ahclkx.gpio3_21", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{"mii1_txclk.gpio3_9", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

#ifdef CONFIG_ANDROID
/* 
 * PWM backlight 
 */
static struct platform_pwm_backlight_data am335x_backlight_data = {
	.pwm_id         = "ehrpwm.0",
	.ch             = 0,
	.max_brightness = 100,
	.dft_brightness = 100,
	.pwm_period_ns  = 31250,
};

static struct platform_device am335x_backlight = {
	.name           = "pwm-backlight",
	.id             = -1,
	.dev		= {
		.platform_data = &am335x_backlight_data,
	},
};

static struct pwmss_platform_data pwm_pdata = {
	.version = PWM_VERSION_1,
};

/* setup haptics */
#define HAPTICS_MAX_FREQ 32000
static void haptics_init(void)
{
	setup_pin_mux(haptics_pin_mux);
	pwm_pdata.chan_attrib[0].max_freq = HAPTICS_MAX_FREQ;
	am33xx_register_ehrpwm(0, &pwm_pdata);
}

static int __init pwm_backlight_init(void)
{
	//am335x_backlight_data.lth_brightness = 59;
	platform_device_register(&am335x_backlight);

	return 0;
}
late_initcall(pwm_backlight_init);
#else

#define VAR_SOM_BACKLIGHT_GPIO GPIO_TO_PIN(0, 2)
static int __init backlight_init(void)
{
	int status;

	setup_pin_mux(gpio_backlight_pin_mux);

	status = gpio_request(VAR_SOM_BACKLIGHT_GPIO, "backlight\n");
	if (status < 0)
		pr_err("Failed to request gpio for backlight");

	gpio_direction_output(VAR_SOM_BACKLIGHT_GPIO, 1);
	gpio_export(VAR_SOM_BACKLIGHT_GPIO, 0);

	return 0;
}
late_initcall(backlight_init);
#endif /* CONFIG_ANDROID */

static void d_can_init(void)
{
	setup_pin_mux(d_can0_pin_mux);
	am33xx_d_can_init(0);
}

static int __init conf_disp_pll(int rate)
{
	struct clk *disp_pll;
	int ret = -EINVAL;

	disp_pll = clk_get(NULL, "dpll_disp_ck");
	if (IS_ERR(disp_pll)) {
		pr_err("Cannot clk_get disp_pll\n");
		goto out;
	}

	ret = clk_set_rate(disp_pll, rate);
	clk_put(disp_pll);
out:
	return ret;
}

static void lcdc_init(void)
{
	struct da8xx_lcdc_platform_data* plcdc_data;

	plcdc_data = &YAOYU_LCD_DATA;
	
	setup_pin_mux(lcdc_pin_mux);

	if (conf_disp_pll(300000000)) {
		pr_info("Failed configure display PLL, not attempting to"
				"register LCDC\n");
		return;
	}

	plcdc_data->get_context_loss_count = omap_pm_get_dev_context_loss_count;

	if (am33xx_register_lcdc(plcdc_data))
		pr_info("Failed to register LCDC device\n");

	return;
}



static void uart_init(void)
{
	setup_pin_mux(uart0_pin_mux);
	setup_pin_mux(uart3_pin_mux);

	omap_serial_init();
}

#define VAR_SOM_GMII1_RST_GPIO   GPIO_TO_PIN(2, 19)

static void rmii1_init(void)
{
	int status;

	omap_mux_init_gpio(VAR_SOM_GMII1_RST_GPIO, OMAP_PIN_OUTPUT);

	status = gpio_request_one(VAR_SOM_GMII1_RST_GPIO,
		GPIOF_OUT_INIT_HIGH, "rmii1_rst");
	if (status) {
		pr_err("Error requesting rmii1 reset gpio: %d\n", status);
	}

	/* Reset RMII1 */
	mdelay(10);
	gpio_set_value(VAR_SOM_GMII1_RST_GPIO, 0);
	mdelay(10);
	gpio_set_value(VAR_SOM_GMII1_RST_GPIO, 1);

	gpio_export(VAR_SOM_GMII1_RST_GPIO, 0);

	setup_pin_mux(rmii1_pin_mux);

	return;
}

#define VAR_SOM_RGMII2_RST_GPIO   GPIO_TO_PIN(3, 10)

static struct gpio rgmii_strapping_gpios[] __initdata = {
	{GPIO_TO_PIN(1, 23), GPIOF_OUT_INIT_HIGH,	"rgmii_phyaddr2"},
	{GPIO_TO_PIN(1, 27), GPIOF_OUT_INIT_HIGH,	"rgmii_mode0"},
	{GPIO_TO_PIN(1, 26), GPIOF_OUT_INIT_HIGH,	"rgmii_mode1"},
	{GPIO_TO_PIN(1, 25), GPIOF_OUT_INIT_HIGH,	"rgmii_mode2"},
	{GPIO_TO_PIN(1, 24), GPIOF_OUT_INIT_HIGH,	"rgmii_mode3"},
	{GPIO_TO_PIN(1, 17), GPIOF_OUT_INIT_HIGH,	"rgmii_clk125_ena"},
};

static void rgmii2_init(void)
{
	int status;

	setup_pin_mux(rgmii2_strapping_pin_mux);

	omap_mux_init_gpio(VAR_SOM_RGMII2_RST_GPIO, OMAP_PIN_OUTPUT);
	status = gpio_request_one(VAR_SOM_RGMII2_RST_GPIO,
		GPIOF_OUT_INIT_HIGH, "rgmii2_rst");
	if (status) {
		pr_err("Error requesting rgmii2 reset gpio: %d\n", status);
	}

	/* Setup RGMII strapping options */
	status = gpio_request_array(rgmii_strapping_gpios,
				 ARRAY_SIZE(rgmii_strapping_gpios));
	if (status)
		pr_err("Error requesting rgmii2 strapping gpios: %d\n", status);

	/* Reset RGMII2 */
	mdelay(10);
	gpio_set_value(VAR_SOM_RGMII2_RST_GPIO, 0);
	mdelay(10);
	gpio_set_value(VAR_SOM_RGMII2_RST_GPIO, 1);

	gpio_free_array(rgmii_strapping_gpios,
				ARRAY_SIZE(rgmii_strapping_gpios));

	gpio_export(VAR_SOM_RGMII2_RST_GPIO, 0);

	setup_pin_mux(rgmii2_pin_mux);

	return;
}

/* NAND partition information
 */
static struct mtd_partition am335x_nand_partitions[] = {
/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "SPL",
		.offset         = 0,			/* Offset = 0x0 */
		.size           = SZ_128K,
	},
	{
		.name           = "SPL.backup1",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x20000 */
		.size           = SZ_128K,
	},
	{
		.name           = "SPL.backup2",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x40000 */
		.size           = SZ_128K,
	},
	{
		.name           = "SPL.backup3",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x60000 */
		.size           = SZ_128K,
	},
	{
		.name           = "U-Boot",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x80000 */
		.size           = 15 * SZ_128K,
	},
	{
		.name           = "U-Boot Env",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x260000 */
		.size           = 1 * SZ_128K,
	},
	{
		.name           = "Ramdisk",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x280000 */
		.size           = 4 * SZ_128K,
	},
	{
		.name           = "Kernel",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x300000 */
		.size           = 40 * SZ_128K,
	},
	{
		.name           = "File System",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x800000 */
		.size           = MTDPART_SIZ_FULL,
	},
};

static struct gpmc_timings am335x_nand_timings = {
	.sync_clk = 0,

	.cs_on = 0,
	.cs_rd_off = 44,
	.cs_wr_off = 44,

	.adv_on = 6,
	.adv_rd_off = 34,
	.adv_wr_off = 44,
	.we_off = 40,
	.oe_off = 54,

	.access = 64,
	.rd_cycle = 82,
	.wr_cycle = 82,

	.wr_access = 40,
	.wr_data_mux_bus = 0,
};

static void som_nand_init(void)
{
	struct omap_nand_platform_data *pdata;
	struct gpmc_devices_info gpmc_device[2] = {
		{ NULL, 0 },
		{ NULL, 0 },
	};

	setup_pin_mux(nand_pin_mux);
	pdata = omap_nand_init(am335x_nand_partitions,
		ARRAY_SIZE(am335x_nand_partitions), 0, 0,
		&am335x_nand_timings);
	if (!pdata)
		return;
	pdata->ecc_opt =OMAP_ECC_BCH8_CODE_HW;
	pdata->elm_used = true;
	gpmc_device[0].pdata = pdata;
	gpmc_device[0].flag = GPMC_DEVICE_NAND;

	omap_init_gpmc(gpmc_device, sizeof(gpmc_device));
	omap_init_elm();
}

/* Setup McASP 0 */
static void mcasp0_init(void)
{
	/* Configure McASP */
	setup_pin_mux(mcasp0_pin_mux);
	am335x_register_mcasp(&var_am335x_som_snd_data0, 0);
	return;
}

/* WLAN */
static void mmc1_wl12xx_init(void)
{
	setup_pin_mux(mmc1_wl12xx_pin_mux);

	am335x_mmc[1].mmc = 2;
	am335x_mmc[1].name = "wl1271";
	am335x_mmc[1].caps = MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD
				| MMC_PM_KEEP_POWER;
	am335x_mmc[1].nonremovable = true;
	am335x_mmc[1].gpio_cd = -EINVAL;
	am335x_mmc[1].gpio_wp = -EINVAL;
	am335x_mmc[1].ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34; /* 3V3 */

	/* mmc will be initialized when mmc0_init is called */
	return;
}

static void uart1_wl12xx_init(void)
{
	setup_pin_mux(uart1_wl12xx_pin_mux);
}

#ifdef CONFIG_TI_ST
/* TI-ST for WL12xx BT */

static int plat_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int plat_kim_resume(struct platform_device *pdev)
{
	return 0;
}

static int plat_kim_chip_enable(struct kim_data_s *kim_data)
{
	gpio_direction_output(kim_data->nshutdown, 0);
	msleep(1);
	gpio_direction_output(kim_data->nshutdown, 1);

	return 0;
}

static int plat_kim_chip_disable(struct kim_data_s *kim_data)
{
	gpio_direction_output(kim_data->nshutdown, 0);

	return 0;
}

static struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = VAR_SOM_BT_PMENA_GPIO,
	.dev_name = "/dev/ttyO1",
	.flow_cntrl = 1,
	.baud_rate = 3000000,
	.suspend = plat_kim_suspend,
	.resume = plat_kim_resume,
	.chip_enable = plat_kim_chip_enable,
	.chip_disable = plat_kim_chip_disable,
};

static struct platform_device wl12xx_device = {
	.name       = "kim",
	.id     = -1,
	.dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static inline void __init var_som_init_btwilink(void)
{
	pr_info("var_som_am33: bt init\n");

	platform_device_register(&wl12xx_device);
	platform_device_register(&btwilink_device);
}
#endif

static void wl12xx_bluetooth_enable(void)
{
#ifndef CONFIG_TI_ST
	int status = gpio_request(var_som_am33_wlan_data.bt_enable_gpio,
			"bt_en\n");
	if (status < 0)
		pr_err("Failed to request gpio for bt_enable");

	pr_info("Configure Bluetooth Enable pin...\n");
	gpio_direction_output(var_som_am33_wlan_data.bt_enable_gpio, 0);
#else
	var_som_init_btwilink();
#endif
}

static int wl12xx_set_power(struct device *dev, int slot, int on, int vdd)
{
	if (on) {
		gpio_set_value(var_som_am33_wlan_data.wlan_enable_gpio, 1);
		mdelay(70);
	}
	else
		gpio_set_value(var_som_am33_wlan_data.wlan_enable_gpio, 0);

	return 0;
}

static void __init wl12xx_init(void)
{
	struct device *dev;
	struct omap_mmc_platform_data *pdata;
	int ret;

	setup_pin_mux(wl12xx_pin_mux_var_som);

	if (get_var_am33_som_rev() >= 3) {
		var_som_am33_wlan_data.board_ref_clock = WL12XX_REFCLOCK_38;
		var_som_am33_wlan_data.board_tcxo_clock = WL12XX_TCXOCLOCK_38_4;
	}

	/* Register WLAN enable pin */
	if (wl12xx_set_platform_data(&var_som_am33_wlan_data))
		pr_err("error setting wl12xx data\n");

	dev = am335x_mmc[1].dev;
	if (!dev) {
		pr_err("wl12xx mmc device initialization failed\n");
		goto out;
	}

	pdata = dev->platform_data;
	if (!pdata) {
		pr_err("Platfrom data of wl12xx device not set\n");
		goto out;
	}

	wl12xx_bluetooth_enable();

	ret = gpio_request_one(var_som_am33_wlan_data.wlan_enable_gpio,
		GPIOF_OUT_INIT_LOW, "wlan_en");
	if (ret) {
		pr_err("Error requesting wlan enable gpio: %d\n", ret);
		goto out;
	}

	pdata->slots[0].set_power = wl12xx_set_power;
out:
	return;
}

static void __init mmc0_init(void)
{
	setup_pin_mux(mmc0_pin_mux);

	omap2_hsmmc_init(am335x_mmc);
	return;
}

#define VAR_SOM_KS8051_PHY_ID       0x00221556
#define VAR_SOM_KS8051_PHY_MASK     0xffffffff

#define VAR_SOM_KS8081_PHY_ID       0x00221560
#define VAR_SOM_KS8081_PHY_MASK     0xffffffff

#define VAR_SOM_KSZ9021_PHY_ID      0x00221611
#define VAR_SOM_KSZ9021_PHY_MASK    0xffffffff

static int var_som_ks8051_phy_fixup(struct phy_device *phydev)
{
	/* override strap options, set RMII mode */
	phy_write(phydev, 0x16, 0x2);

	return 0;
}

static int var_som_ks8081_phy_fixup(struct phy_device *phydev)
{
	/* override strap options, set RMII mode */
	phy_write(phydev, 0x16, 0x2);

	return 0;
}

static int var_som_ksz9021_phy_fixup(struct phy_device *phydev)
{
	/* Fine-tune clock and control pad skew */
	phy_write(phydev, 0xb, 0x8104);
	phy_write(phydev, 0xc, 0xA097);

	/* Fine-tune RX data pad skew */
	phy_write(phydev, 0xb, 0x8105);
	phy_write(phydev, 0xc, 0);

	return 0;
}

static void ethernet_init(void)
{
	/* Setup fallback MAC addresses, used only if eFuse MACID is invalid.
	 */
	am335x_mac_addr[0][0]=0xF8;
	am335x_mac_addr[0][1]=0xdc;
	am335x_mac_addr[0][2]=0x7a;
	am335x_mac_addr[0][3]=0x00;
	am335x_mac_addr[0][4]=0x11;
	am335x_mac_addr[0][5]=0x22;

	am335x_mac_addr[1][0]=0xF8;
	am335x_mac_addr[1][1]=0xdc;
	am335x_mac_addr[1][2]=0x7a;
	am335x_mac_addr[1][3]=0x00;
	am335x_mac_addr[1][4]=0x11;
	am335x_mac_addr[1][5]=0x23;
	am33xx_cpsw_macidfillup(am335x_mac_addr[0],	am335x_mac_addr[1]);

	phy_register_fixup_for_uid(VAR_SOM_KS8051_PHY_ID, VAR_SOM_KS8051_PHY_MASK,
				   var_som_ks8051_phy_fixup);

	phy_register_fixup_for_uid(VAR_SOM_KS8081_PHY_ID, VAR_SOM_KS8081_PHY_MASK,
				   var_som_ks8081_phy_fixup);

	phy_register_fixup_for_uid(VAR_SOM_KSZ9021_PHY_ID, VAR_SOM_KSZ9021_PHY_MASK,
				   var_som_ksz9021_phy_fixup);

	am33xx_cpsw_init(AM33XX_CPSW_MODE_VAR, "0:01", "0:07");
}

static struct regulator_init_data am335x_dummy = {
	.constraints.always_on	= true,
};

static struct regulator_consumer_supply am335x_vdd1_supply[] = {
	REGULATOR_SUPPLY("vdd_mpu", NULL),
};

static struct regulator_init_data am335x_vdd1 = {
	.constraints = {
		.min_uV			= 600000,
		.max_uV			= 1500000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(am335x_vdd1_supply),
	.consumer_supplies	= am335x_vdd1_supply,
};

static struct regulator_consumer_supply am335x_vdd2_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_init_data am335x_vdd2 = {
	.constraints = {
		.min_uV			= 600000,
		.max_uV			= 1500000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(am335x_vdd2_supply),
	.consumer_supplies	= am335x_vdd2_supply,
};

static struct tps65910_board am335x_tps65910_info = {
	.tps65910_pmic_init_data[TPS65910_REG_VRTC]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VIO]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDD1]	= &am335x_vdd1,
	.tps65910_pmic_init_data[TPS65910_REG_VDD2]	= &am335x_vdd2,
	.tps65910_pmic_init_data[TPS65910_REG_VDD3]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDIG1]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDIG2]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VPLL]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDAC]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX1]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX2]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX33]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VMMC]	= &am335x_dummy,
};

#define AM33XX_VDD_CORE_OPP50_UV		1100000
#define AM33XX_OPP120_FREQ		600000000
#define AM33XX_OPPTURBO_FREQ		720000000

#define AM33XX_ES2_0_VDD_CORE_OPP50_UV	950000
#define AM33XX_ES2_0_OPP120_FREQ	720000000
#define AM33XX_ES2_0_OPPTURBO_FREQ	800000000
#define AM33XX_ES2_0_OPPNITRO_FREQ	1000000000

#define AM33XX_ES2_1_VDD_CORE_OPP50_UV	950000
#define AM33XX_ES2_1_OPP120_FREQ	720000000
#define AM33XX_ES2_1_OPPTURBO_FREQ	800000000
#define AM33XX_ES2_1_OPPNITRO_FREQ	1000000000

static int am335x_opp_update(void)
{
	u32 rev;
	int voltage_uv = 0;
	struct device *core_dev, *mpu_dev;
	struct regulator *core_reg;

	core_dev = omap_device_get_by_hwmod_name("l3_main");
	mpu_dev = omap_device_get_by_hwmod_name("mpu");

	if (!mpu_dev || !core_dev) {
		pr_err("%s: Aiee.. no mpu/core devices? %p %p\n", __func__,
		       mpu_dev, core_dev);
		return -1;
	}

	core_reg = regulator_get(core_dev, "vdd_core");
	if (IS_ERR(core_reg)) {
		pr_err("%s: unable to get core regulator\n", __func__);
		return -1;
	}

	/*
	 * Ensure physical regulator is present.
	 * (e.g. could be dummy regulator.)
	 */
	voltage_uv = regulator_get_voltage(core_reg);
	if (voltage_uv < 0) {
		pr_err("%s: physical regulator not present for core" \
		       "(%d)\n", __func__, voltage_uv);
		regulator_put(core_reg);
		return -1;
	}

	pr_debug("%s: core regulator value %d\n", __func__, voltage_uv);
	if (voltage_uv > 0) {
		rev = omap_rev();
		switch (rev) {
		case AM335X_REV_ES1_0:
			if (voltage_uv <= AM33XX_VDD_CORE_OPP50_UV) {
				/*
				 * disable the higher freqs - we dont care about
				 * the results
				 */
				opp_disable(mpu_dev, AM33XX_OPP120_FREQ);
				opp_disable(mpu_dev, AM33XX_OPPTURBO_FREQ);
			}
			break;
		case AM335X_REV_ES2_0:
			if (voltage_uv <= AM33XX_ES2_0_VDD_CORE_OPP50_UV) {
				/*
				 * disable the higher freqs - we dont care about
				 * the results
				 */
				opp_disable(mpu_dev,
					    AM33XX_ES2_0_OPP120_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_0_OPPTURBO_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_0_OPPNITRO_FREQ);
			}
			break;
		case AM335X_REV_ES2_1:
		/* FALLTHROUGH */
		default:
			if (voltage_uv <= AM33XX_ES2_1_VDD_CORE_OPP50_UV) {
				/*
				 * disable the higher freqs - we dont care about
				 * the results
				 */
				opp_disable(mpu_dev,
					    AM33XX_ES2_1_OPP120_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_1_OPPTURBO_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_1_OPPNITRO_FREQ);
			}
			break;
		}
	}

	return 0;
}
late_initcall(am335x_opp_update);


static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_ULPI,
	/*
	 * mode[0:3] = USB0PORT's mode
	 * mode[4:7] = USB1PORT's mode
	 */
	.mode           = (MUSB_PERIPHERAL << 4) | MUSB_HOST,
	.power		= 500,
	.instances	= 1,
};

#define MST3000_TSC_CTW_IRQ_GPIO 	GPIO_TO_PIN(0, 20)


struct mst3000_charger_pdata charger_platform_data = {
		.pg_pin = GPIO_TO_PIN(2, 1),
		.ce_pin = GPIO_TO_PIN(1, 27),
};

static struct i2c_board_info __initdata var_som_i2c1_boardinfo[] = {
		{
				I2C_BOARD_INFO("tps65910", TPS65910_I2C_ID1),
				.platform_data  = &am335x_tps65910_info,
		},
		{
				I2C_BOARD_INFO("tlv320aic3x", 0x1b),
		},
		{
				/* bq27421 Gas Gauge */
				I2C_BOARD_INFO("bq274xx", 0x55),
		},
		{
				/* bq24196 battery charger */
				I2C_BOARD_INFO("bq24196", 0x6b),
				.platform_data = &charger_platform_data,
		},
		{
				/* External RTC on main board */
				I2C_BOARD_INFO("rx8581", 0x51),
		},
};

static struct i2c_board_info __initdata var_som_i2c0_boardinfo[] = {
		{
				/* SSD2543 Touch panel */
				I2C_BOARD_INFO("ssd2543-ts", 0x48),
		},
//		{
//				.flags = I2C_CLIENT_WAKE,
//				.irq = OMAP_GPIO_IRQ(MST3000_TSC_CTW_IRQ_GPIO),
//		},
};

static void i2c1_init(void)
{
	setup_pin_mux(i2c1_pin_mux);

	omap_register_i2c_bus(2, 100, var_som_i2c1_boardinfo,
			ARRAY_SIZE(var_som_i2c1_boardinfo));
	return;
}

static void i2c0_init(void)
{
	setup_pin_mux(i2c0_pin_mux);

	omap_register_i2c_bus(1, 100, var_som_i2c0_boardinfo,
			ARRAY_SIZE(var_som_i2c0_boardinfo));
	return;
}

static struct spi_board_info mst3000_spi1_slave_info[] = {
		{
				.modalias = "spidev",
				.irq = -1,
				.max_speed_hz = 2000000,
				.bus_num = 2,
				.chip_select = 0,
		}
};

static void spi1_init(void)
{
	spi_register_board_info(mst3000_spi1_slave_info,
			ARRAY_SIZE(mst3000_spi1_slave_info));

}

/* Enable clkout1 */
static struct pinmux_config clkout1_pin_mux[] = {
	{"xdma_event_intr0.clkout1", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static void __init clkout1_enable(void)
{
	setup_pin_mux(clkout1_pin_mux);
}

static struct resource am33xx_cpuidle_resources[] = {
	{
		.start		= AM33XX_EMIF0_BASE,
		.end		= AM33XX_EMIF0_BASE + SZ_32K - 1,
		.flags		= IORESOURCE_MEM,
	},
};

/* AM33XX devices support DDR2 power down */
static struct am33xx_cpuidle_config am33xx_cpuidle_pdata = {
	.ddr2_pdown	= 1,
};

static struct platform_device am33xx_cpuidle_device = {
	.name			= "cpuidle-am33xx",
	.num_resources		= ARRAY_SIZE(am33xx_cpuidle_resources),
	.resource		= am33xx_cpuidle_resources,
	.dev = {
		.platform_data	= &am33xx_cpuidle_pdata,
	},
};

static void __init am33xx_cpuidle_init(void)
{
	extern void __iomem * __init am33xx_get_mem_ctlr(void);
	int ret;

	am33xx_cpuidle_pdata.emif_base = am33xx_get_mem_ctlr();

	ret = platform_device_register(&am33xx_cpuidle_device);

	if (ret)
		pr_warning("AM33XX cpuidle registration failed\n");
}

#ifdef CONFIG_ANDROID
static void sgx_init(void)
{
	if (omap3_has_sgx()) {
		am33xx_gpu_init();
	}
}
#endif


static struct pinmux_config bat_adc_pin_mux[] = {
		{"ain5.ain5",           OMAP_MUX_MODE0 | AM33XX_INPUT_EN},
		{"vrefp.vrefp",         OMAP_MUX_MODE0 | AM33XX_INPUT_EN},
		{"vrefn.vrefn",         OMAP_MUX_MODE0 | AM33XX_INPUT_EN},
		{NULL, 0},
};

static void bat_adc_init(void)
{
	int err;

	setup_pin_mux(bat_adc_pin_mux);
	err = am33xx_register_mfd_tscadc(&battery_tscadc);
	if (err)
		pr_err("failed to register battery adc device\n");
}

static void setup_mst3000(void)
{
	printk("setup_mst3000 \n");
	setup_pin_mux(mst3000_battery_pinmux);
	setup_pin_mux(mst3000_subcpu_pinmux);
	mst3000_keys_init(0, 0);
    //mst3000_battery_charger_init(0, 0);
    
    bat_adc_init();

    gpio_request(CHG_EN, "charger_enable");
	gpio_direction_output(CHG_EN, 0);

	gpio_request(TOUCH_RESET, "touch_reset");
	gpio_direction_output(TOUCH_RESET, 1);

	spi1_init();

}

static void mst3000_poweroff(void)
{
	mst3000_charger_poweroff();
	gpio_direction_output(PWR_CNTRL, 0);
}

static void __init var_am335x_som_init(void)
{
	am33xx_cpuidle_init();
	am33xx_mux_init(board_mux);

	gpio_request(PWR_CNTRL, "power_control");
	gpio_direction_output(PWR_CNTRL, 1);


	uart_init();
	printk ("Variscite AM33 SOM revision %s detected\n",
			get_var_am33_som_rev_str());

	if (get_var_am33_som_rev() < 2)
		pr_err("CRITICAL: SOM REVISION %s IS NOT SUPPORTED !!!\n", 
				get_var_am33_som_rev_str());

	clkout1_enable(); /* Required by Audio codec */
	mmc1_wl12xx_init();
	mmc0_init();
	uart1_wl12xx_init();
	wl12xx_init();
	som_nand_init();
	lcdc_init();
	//mfd_tscadc_init();
	mcasp0_init();
	rmii1_init();
	//rgmii2_init();
	ethernet_init();
	i2c1_init();
	i2c0_init();
#ifdef CONFIG_ANDROID
	haptics_init(); /* use by PWM backlight */
	sgx_init();
#endif	
	usb_musb_init(&musb_board_data);
	d_can_init();

	/* Create an alias for icss clock */
	if (clk_add_alias("pruss", NULL, "pruss_uart_gclk", NULL))
		pr_warn("failed to create an alias: icss_uart_gclk --> pruss\n");
	/* Create an alias for gfx/sgx clock */
	if (clk_add_alias("sgx_ck", NULL, "gfx_fclk", NULL))
		pr_warn("failed to create an alias: gfx_fclk --> sgx_ck\n");

	setup_mst3000();

	pm_power_off = mst3000_poweroff;
}

static void __init var_am335x_som_map_io(void)
{
	omap2_set_globals_am33xx();
	omapam33xx_map_common_io();
}

MACHINE_START(VAR_SOM_AM33, "VAR-SOM-AM33")
	/* Maintainer: Uri Yosef <uri.y@variscite.com> */
	.atag_offset	= 0x100,
	.map_io		= var_am335x_som_map_io,
	.init_early	= am33xx_init_early,
	.init_irq	= ti81xx_init_irq,
	.handle_irq     = omap3_intc_handle_irq,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= var_am335x_som_init,
MACHINE_END
