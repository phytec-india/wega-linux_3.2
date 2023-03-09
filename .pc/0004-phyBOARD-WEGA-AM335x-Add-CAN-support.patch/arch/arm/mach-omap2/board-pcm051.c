/*
 * Code for phyCORE-AM335x.
 *
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
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/mfd/tps65217.h>
#include <linux/reboot.h>
#include <linux/opp.h>

#include <mach/hardware.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/asp.h>

#include <plat/omap_device.h>
#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/nand.h>

#include "board-flash.h"
#include "mux.h"
#include "common.h"
#include "am33xx_generic.h"
#include "am33xx_devices.h"

/* Module pin mux for tps65910 irq */
static struct pinmux_config pmic_irq_pin_mux[] = {
	{"mii1_rxdv.gpio3_4",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

/* Pin mux for mmc0 cd */
static struct pinmux_config mmc0_cd_pin_mux[] = {
	{"spi0_cs1.mmc0_sdcd",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

/* Pin mux for uart1 */
static struct pinmux_config uart1_pin_mux[] = {
	{"uart1_txd.uart1_txd", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL},
	{"uart1_rxd.uart1_rxd", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"uart1_ctsn.uart1_ctsn", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"uart1_rtsn.uart1_rtsn", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT},
	{NULL, 0},
};

static struct pinmux_config usb_oc_pins_mux[] = {
	{"mcasp0_aclkr.gpio3_18",	OMAP_MUX_MODE7 | AM33XX_PULL_ENBL |
					AM33XX_PIN_INPUT_PULLUP},
	{"mcasp0_fsr.gpio3_19",		OMAP_MUX_MODE7 | AM33XX_PULL_ENBL |
					AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct gpmc_timings am335x_nand_timings = {

	/* granularity of 10 is sufficient because of calculations */
	.sync_clk = 0,

	.cs_on = 0,
	.cs_rd_off = 30,
	.cs_wr_off = 30,

	.adv_on = 0,
	.adv_rd_off = 30,
	.adv_wr_off = 30,

	.oe_on = 10,
	.we_off = 20,
	.oe_off = 30,

	.access = 30,
	.rd_cycle = 30,
	.wr_cycle = 30,

	.cs_cycle_delay = 50,
	.cs_delay_en = 1,
	.wr_access = 30,
	.wr_data_mux_bus = 0,
};

static struct at24_platform_data am335x_baseboard_eeprom_info = {
	.byte_len	= (32*1024) / 8,
	.page_size	= 32,
	.flags		= AT24_FLAG_ADDR16,
};

static struct i2c_board_info __initdata pcm051_i2c_eeprom_boardinfo[] = {
	{
		I2C_BOARD_INFO("24c32", 0x52),
		.platform_data  = &am335x_baseboard_eeprom_info,
	},
};

static struct i2c_board_info __initdata pcm051_i2c_rtc_boardinfo[] = {
	{
		I2C_BOARD_INFO("rv4162c7", 0x68),
	},
};

static void pcm051_modul_init(void)
{
	am335x_nand_init(&am335x_nand_timings);

	setup_pin_mux(pmic_irq_pin_mux);
	am33xx_tps65910_init(1, GPIO_TO_PIN(3, 4));

	i2c_register_board_info(1, pcm051_i2c_eeprom_boardinfo, 1);
	i2c_register_board_info(1, pcm051_i2c_rtc_boardinfo, 1);
}

static void wega_board(void)
{
	setup_pin_mux(uart1_pin_mux);

	setup_pin_mux(mmc0_cd_pin_mux);
	am335x_mmc0_init(GPIO_TO_PIN(0, 6), -1);

	am33xx_rmii1_mii2_init("0:00", "0:01");

	setup_pin_mux(usb_oc_pins_mux);
	am33xx_usb0_otg_usb1_host_init();
}

static void __init pcm051_init(void)
{
	am33xx_cpuidle_init();
	am33xx_mux_init(NULL);
	omap_serial_init();
	am33xx_clkout1_enable();
	omap_sdrc_init(NULL, NULL);
	/* Create an alias for icss clock */
	if (clk_add_alias("pruss", NULL, "pruss_uart_gclk", NULL))
		pr_warn("failed to create an alias: pruss_uart_gclk --> pruss\n");
	/* Create an alias for gfx/sgx clock */
	if (clk_add_alias("sgx_ck", NULL, "gfx_fclk", NULL))
		pr_warn("failed to create an alias: gfx_fclk --> sgx_ck\n");

	pcm051_modul_init();
	wega_board();

	omap_register_i2c_bus(1, 100, NULL, 0);

}

MACHINE_START(PCM051, "pcm051")
	/* Maintainer: PHYTEC */
	.atag_offset	= 0x100,
	.map_io		= am33xx_map_io_set_globals,
	.init_early	= am33xx_init_early,
	.init_irq	= ti81xx_init_irq,
	.handle_irq	= omap3_intc_handle_irq,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= pcm051_init,
MACHINE_END
