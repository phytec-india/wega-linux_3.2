/*
 * Copyright (C) 2013 Phytec Messtechnik GmbH
 *
 * Based on mach-omap2/board-am335xevm.c
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
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/mfd/tps65910.h>
#include <linux/mfd/ti_tscadc.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/pwm/pwm.h>
#include <linux/pwm_backlight.h>
#include <plat/omap_device.h>
#include <plat/mmc.h>
#include <plat/nand.h>
#include <plat/usb.h>

#include "board-flash.h"
#include "am33xx_generic.h"
#include "cpuidle33xx.h"
#include "devices.h"
#include "hsmmc.h"
#include "mux.h"

/* Pin mux for nand flash module */
static struct pinmux_config nand_pin_mux[] = {
	{"gpmc_ad0.gpmc_ad0",     OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad1.gpmc_ad1",     OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad2.gpmc_ad2",     OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad3.gpmc_ad3",     OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad4.gpmc_ad4",     OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad5.gpmc_ad5",     OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad6.gpmc_ad6",     OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad7.gpmc_ad7",     OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wait0.gpmc_wait0", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn0.gpmc_csn0",   OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_advn_ale.gpmc_advn_ale",  OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_oen_ren.gpmc_oen_ren",    OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_wen.gpmc_wen",     OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_ben0_cle.gpmc_ben0_cle",  OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{NULL, 0},
};

void am335x_nand_init(struct gpmc_timings *am335x_nand_timings)
{
	struct omap_nand_platform_data *pdata;
	struct gpmc_devices_info gpmc_device[2] = {
		{ NULL, 0 },
		{ NULL, 0 },
	};

	setup_pin_mux(nand_pin_mux);
	pdata = omap_nand_init(NULL, 0, 0, 0, am335x_nand_timings);
	if (!pdata)
		return;
	pdata->ecc_opt = OMAP_ECC_BCH8_CODE_HW;
	pdata->elm_used = true;
	gpmc_device[0].pdata = pdata;
	gpmc_device[0].flag = GPMC_DEVICE_NAND;

	omap_init_gpmc(gpmc_device, sizeof(gpmc_device));
	omap_init_elm();
}

/* Module pin mux for SPI fash */
static struct pinmux_config spi0_pin_mux[] = {
	{"spi0_sclk.spi0_sclk", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL
							| AM33XX_INPUT_EN},
	{"spi0_d0.spi0_d0", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP
							| AM33XX_INPUT_EN},
	{"spi0_d1.spi0_d1", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL
							| AM33XX_INPUT_EN},
	{"spi0_cs0.spi0_cs0", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP
							| AM33XX_INPUT_EN},
	{NULL, 0},
};

void __init am335x_spi0_init(
		const struct spi_board_info *am335x_spi0_slave_info,
		unsigned size)
{
	setup_pin_mux(spi0_pin_mux);
	spi_register_board_info(am335x_spi0_slave_info, size);
}

static struct omap2_hsmmc_info am33xx_mmc0[] __initdata = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34, /* 3V3 */
	},
	{}
};

/* Module pin mux for mmc0 */
static struct pinmux_config mmc0_pin_mux[] = {
	{"mmc0_dat3.mmc0_dat3", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat2.mmc0_dat2", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat1.mmc0_dat1", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat0.mmc0_dat0", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_clk.mmc0_clk",   OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_cmd.mmc0_cmd",   OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

void __init am335x_mmc0_init(int gpio_cd, int gpio_wp)
{
	setup_pin_mux(mmc0_pin_mux);
	if (gpio_cd > 0)
		am33xx_mmc0[0].gpio_cd = gpio_cd;
	if (gpio_wp > 0)
		am33xx_mmc0[0].gpio_wp = gpio_wp;
	omap2_hsmmc_init(am33xx_mmc0);

	return;
}

/* Module pin mux for rmii1 */
static struct pinmux_config rmii1_pin_mux[] = {
	{"mii1_crs.rmii1_crs_dv", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxerr.mii1_rxerr", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txen.mii1_txen", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_txd1.mii1_txd1", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_txd0.mii1_txd0", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_rxd1.mii1_rxd1", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd0.mii1_rxd0", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"rmii1_refclk.rmii1_refclk", OMAP_MUX_MODE0 |
					AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

void __init am33xx_rmii1_mii2_init(unsigned char *phy_id0,
						unsigned char *phy_id1)
{
	setup_pin_mux(rmii1_pin_mux);

	am33xx_cpsw_init(AM33XX_CPSW_MODE_RMII1_MII2, phy_id0, phy_id1);
}

/* pinmux for usb0 drvvbus */
static struct pinmux_config usb0_pin_mux[] = {
	{"usb0_drvvbus.usb0_drvvbus", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* pinmux for usb1 drvvbus */
static struct pinmux_config usb1_pin_mux[] = {
	{"usb1_drvvbus.usb1_drvvbus", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static struct omap_musb_board_data am33xx_musb_board_data = {
	.interface_type = MUSB_INTERFACE_ULPI,
	/*
	* mode[0:3] = USB0PORT's mode
	* mode[4:7] = USB1PORT's mode
	* USB0 in OTG mode and USB1 in host mode.
	*/
	.mode		= (MUSB_HOST << 4) | MUSB_OTG,
	.power		= 500,
	.instances	= 1,
};

void __init am33xx_usb0_otg_usb1_host_init(void)
{
	setup_pin_mux(usb0_pin_mux);
	setup_pin_mux(usb1_pin_mux);

	usb_musb_init(&am33xx_musb_board_data);
}


static struct resource am33xx_cpuidle_resources[] = {
	{
		.start		= AM33XX_EMIF0_BASE,
		.end		= AM33XX_EMIF0_BASE + SZ_32K - 1,
		.flags		= IORESOURCE_MEM,
	},
};

static struct pinmux_config clkout1_pin_mux[] = {
	{"xdma_event_intr0.clkout1", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

void __init am33xx_clkout1_enable(void)
{
	struct clk *ck_32;

	ck_32 = clk_get(NULL, "clkout1_ck");
	if (IS_ERR(ck_32)) {
		pr_err("Cannot clk_get ck_32\n");
		return;
	}

	clk_enable(ck_32);

	setup_pin_mux(clkout1_pin_mux);
}

/* AM33XX devices support DDR2 power down */
static struct am33xx_cpuidle_config am33xx_cpuidle_pdata = {
	.ddr2_pdown	= 1,
};

static struct platform_device am33xx_cpuidle_device = {
	.name			= "cpuidle-am33xx",
	.num_resources		= ARRAY_SIZE(am33xx_cpuidle_resources),
	.resource		= am33xx_cpuidle_resources,
	.dev = {
		.platform_data  = &am33xx_cpuidle_pdata,
	},
};

void __init am33xx_cpuidle_init(void)
{
	int ret;

	am33xx_cpuidle_pdata.emif_base = am33xx_get_mem_ctlr();

	ret = platform_device_register(&am33xx_cpuidle_device);

	if (ret)
		pr_warning("AM33XX cpuidle registration failed\n");
}

static struct regulator_init_data am335x_dummy = {
	.constraints.always_on  = true,
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

static struct tps65910_board am335x_tps65910_info = {
	.tps65910_pmic_init_data[TPS65910_REG_VRTC]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VIO]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDD1]	= &am335x_vdd1,
	.tps65910_pmic_init_data[TPS65910_REG_VDD2]	= &am335x_dummy,
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

static struct platform_pwm_backlight_data am335x_backlight_data_ecap2 = {
	.pwm_id		= "ecap.2",
	.ch		= -1,
	.max_brightness = 100,
	.dft_brightness = 100,
	.pwm_period_ns	= 100000,
};

/* Setup pwm-backlight */
static struct platform_device am335x_backlight_ecap2 = {
	.name		= "pwm-backlight",
	.id		= -1,
	.dev		= {
		.platform_data	= &am335x_backlight_data_ecap2,
	}
};

static struct pwmss_platform_data pwm_pdata = {
	.version = PWM_VERSION_1
};

int __init am33xx_ecap2_init(unsigned int dft_brightness)
{
	am33xx_register_ecap(2, &pwm_pdata);
	am335x_backlight_data_ecap2.dft_brightness = dft_brightness;
	platform_device_register(&am335x_backlight_ecap2);

	return 0;
}

static struct i2c_board_info __initdata am335x_tps65910_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps65910", 0x2D),
		.platform_data  = &am335x_tps65910_info,
	},
};

int __init am33xx_tps65910_init(int i2c_busnum, int gpio_irq)
{
	int ret;

	if (gpio_irq >= 0) {
		ret = gpio_request_one(gpio_irq, GPIOF_IN, "rtc-tps65910-irq");
		if (ret)
			return ret;

		am335x_tps65910_info.irq = OMAP_GPIO_IRQ(gpio_irq);
	}

	i2c_register_board_info(i2c_busnum, am335x_tps65910_boardinfo, 1);

	return 0;
}
