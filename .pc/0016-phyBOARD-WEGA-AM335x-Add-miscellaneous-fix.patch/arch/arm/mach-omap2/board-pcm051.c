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
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
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

#include <video/da8xx-fb.h>
#include <plat/lcdc.h>

#include "board-flash.h"
#include "mux.h"
#include "common.h"
#include "cm33xx.h"
#include "am33xx_generic.h"
#include "am33xx_devices.h"
#include "devices.h"

/* TSc controller */
#include <linux/input/ti_tsc.h>
#include <linux/mfd/ti_tscadc.h>
#include <linux/platform_data/ti_adc.h>

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

static struct pinmux_config d_can1_pin_mux[] = {
	{"uart0_ctsn.d_can1_tx", OMAP_MUX_MODE2 | AM33XX_PULL_ENBL},
	{"uart0_rtsn.d_can1_rx", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config led_pin_mux[] = {
	{"mii1_txclk.gpio3_9", OMAP_MUX_MODE7 | AM33XX_PULL_ENBL},
	{NULL, 0},
};

static struct pinmux_config btn_pin_mux[] = {
	{"xdma_event_intr1.gpio0_20", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{"ecap0_in_pwm0_out.gpio0_7", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{"mii1_rxclk.gpio3_10", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
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
	{"gpmc_ad8.lcd_data16",		OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad9.lcd_data17",		OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad10.lcd_data18",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad11.lcd_data19",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad12.lcd_data20",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad13.lcd_data21",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad14.lcd_data22",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad15.lcd_data23",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"lcd_vsync.lcd_vsync",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_hsync.lcd_hsync",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_pclk.lcd_pclk",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_ac_bias_en.lcd_ac_bias_en", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* ecap2 pinmux */
static struct pinmux_config ecap2_pin_mux[] = {
	{"mcasp0_ahclkr.ecap2_in_pwm2_out", OMAP_MUX_MODE4 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* Module pin mux for mcasp0 */
static struct pinmux_config mcasp0_pin_mux[] = {
	{"mcasp0_aclkx.mcasp0_aclkx", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mcasp0_fsx.mcasp0_fsx", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mcasp0_axr0.mcasp0_axr0", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mcasp0_axr1.mcasp0_axr1", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mcasp0_ahclkx.mcasp0_ahclkx", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
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

static struct i2c_board_info __initdata pcm051_i2c_audio_codec_boardinfo[] = {
	{
		I2C_BOARD_INFO("tlv320aic3007", 0x18),
	},
};

static struct gpio_led gpio_leds[] = {
	{
		.name			= "peb_eval_01:green:led3",
		.default_trigger	= "none",
		.gpio			= 105,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data  = &gpio_led_info,
	},
};

static struct i2c_board_info __initdata pcm051_peb_av_01_boardinfo[] = {
	{
		I2C_BOARD_INFO("tda998x", 0x34),
	},
};

static const struct display_panel disp_panel = {
	WVGA,
	32,
	32,
	COLOR_ACTIVE,
};

static struct lcd_ctrl_config lcd_cfg = {
	&disp_panel,
	.ac_bias		= 40,
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

static struct da8xx_lcdc_platform_data lcdc_pdata[] = {
	{
		.manu_name		= "genericDVI",
		.controller_data	= &lcd_cfg,
		.type			= "DVI_XGA",
	}, {
		.manu_name		= "genericDVI",
		.controller_data	= &lcd_cfg,
		.type			= "DVI_720P",
	}, {
		.manu_name		= "HTdisplay",
		.controller_data	= &lcd_cfg,
		.type			= "HT_HT800070I",
	}, {
		.manu_name		= "ZQdisplay",
		.controller_data	= &lcd_cfg,
		.type			= "ZQ_ZQ3506",
	}, {
		.manu_name		= "NEC Display model",
		.controller_data	= &lcd_cfg,
		.type			= "NL4827HC19",
	},
};

static struct da8xx_lcdc_selection_platform_data lcdc_selection_pdata = {
	.entries_ptr = lcdc_pdata,
	.entries_cnt = ARRAY_SIZE(lcdc_pdata)
};

/* TSc platform data */
static struct tsc_data am335x_touchscreen_data = {
	.wires	= 4,
	.x_plate_resistance = 200,
	.steps_to_configure = 5,
};

static u8 am335x_iis_serializer_direction0[] = {
	RX_MODE,	TX_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
};

static struct snd_platform_data pcm051_snd_data0 = {
	.tx_dma_offset  = 0x46000000,   /* McASP0 */
	.rx_dma_offset  = 0x46000000,
	.op_mode        = DAVINCI_MCASP_IIS_MODE,
	.num_serializer = ARRAY_SIZE(am335x_iis_serializer_direction0),
	.tdm_slots      = 2,
	.serial_dir     = am335x_iis_serializer_direction0,
	.asp_chan_q     = EVENTQ_2, /* davinci-mcsap driver does not use it */
	.version        = MCASP_VERSION_3,
	.txnumevt       = 1,
	.rxnumevt       = 1,
};

static struct mfd_tscadc_board tscadc = {
	.tsc_init	= &am335x_touchscreen_data,
};

/* TSc initialization */
static void am335x_tsc_init(void)
{
	int err;

	err = am33xx_register_mfd_tscadc(&tscadc);
	if (err)
		pr_err("failed to register touchscreen device\n");
}

static void pcm051_modul_init(void)
{
	am335x_nand_init(&am335x_nand_timings);

	am33xx_tps65910_init(1, -1);

	i2c_register_board_info(1, pcm051_i2c_eeprom_boardinfo, 1);
	i2c_register_board_info(1, pcm051_i2c_rtc_boardinfo, 1);
	i2c_register_board_info(1, pcm051_i2c_audio_codec_boardinfo, 1);
}

static void wega_board(void)
{
	setup_pin_mux(uart1_pin_mux);

	setup_pin_mux(mmc0_cd_pin_mux);
	am335x_mmc0_init(GPIO_TO_PIN(0, 6), -1);

	am33xx_rmii1_mii2_init("0:00", "0:01");

	setup_pin_mux(usb_oc_pins_mux);
	am33xx_usb0_otg_usb1_host_init();

	setup_pin_mux(d_can1_pin_mux);
	am33xx_d_can_init(1);

	am335x_tsc_init();
}

static void __init peb_eval_01_init(void)
{
	setup_pin_mux(led_pin_mux);
	setup_pin_mux(btn_pin_mux);
	platform_device_register(&leds_gpio);
}

static void __init peb_av_01(void)
{
	int val;

	setup_pin_mux(lcdc_pin_mux);

	if (conf_disp_pll(300000000)) {
		pr_info("Failed configure display PLL, not attempting to"
			"register LCDC\n");
		return;
	}

	writel(0x4b, AM33XX_CM_SSC_DELTAMSTEP_DPLL_DISP);
	writel(0x44e, AM33XX_CM_SSC_MODFREQDIV_DPLL_DISP);
	val = readl(AM33XX_CM_CLKMODE_DPLL_DISP);
	val |= 0x1000;
	writel(val, AM33XX_CM_CLKMODE_DPLL_DISP);

	am33xx_register_lcdc(&lcdc_selection_pdata);

	i2c_register_board_info(1, pcm051_peb_av_01_boardinfo, 1);
}

/* Setup McASP 0 (Multichannel Audio Serial Port) */
static void mcasp0_init(int evm_id, int profile)
{
	/* Configure McASP */
	setup_pin_mux(mcasp0_pin_mux);
	am335x_register_mcasp(&pcm051_snd_data0, 0);
	return;
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
	peb_eval_01_init();
	peb_av_01();

	omap_register_i2c_bus(1, 100, NULL, 0);
	mcasp0_init(0, 0);
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
