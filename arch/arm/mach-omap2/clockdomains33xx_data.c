/*
 * AM33XX Clock Domain data.
 *
 * Copyright (C) 2011 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/io.h>

#include "prcm44xx.h"
#include "clockdomain.h"
#include "cm.h"
#include "cm33xx.h"
#include "cm-regbits-33xx.h"

static struct clockdomain l4ls_am33xx_clkdm = {
	.name		= "l4ls_clkdm",
	.pwrdm		= { .name = "per_pwrdm" },
	.cm_inst	= AM33XX_CM_PER_MOD,
	.prcm_partition	= AM33XX_PRM_PARTITION,
	.clkdm_offs	= AM33XX_CM_PER_L4LS_CLKSTCTRL_OFFSET,
	.clktrctrl_mask	= AM33XX_CLKTRCTRL_MASK,
	.flags		= (CLKDM_CAN_SWSUP | CLKDM_NO_AUTODEPS),
};

static struct clockdomain l3s_am33xx_clkdm = {
	.name		= "l3s_clkdm",
	.pwrdm		= { .name = "per_pwrdm" },
	.cm_inst	= AM33XX_CM_PER_MOD,
	.prcm_partition	= AM33XX_PRM_PARTITION,
	.clkdm_offs	= AM33XX_CM_PER_L3S_CLKSTCTRL_OFFSET,
	.clktrctrl_mask	= AM33XX_CLKTRCTRL_MASK,
	.flags		= (CLKDM_CAN_SWSUP | CLKDM_NO_AUTODEPS),
};

static struct clockdomain l4fw_am33xx_clkdm = {
	.name		= "l4fw_clkdm",
	.pwrdm		= { .name = "per_pwrdm" },
	.cm_inst	= AM33XX_CM_PER_MOD,
	.prcm_partition	= AM33XX_PRM_PARTITION,
	.clkdm_offs	= AM33XX_CM_PER_L4FW_CLKSTCTRL_OFFSET,
	.clktrctrl_mask	= AM33XX_CLKTRCTRL_MASK,
	.flags		= (CLKDM_CAN_SWSUP | CLKDM_NO_AUTODEPS),
};

static struct clockdomain l3_am33xx_clkdm = {
	.name		= "l3_clkdm",
	.pwrdm		= { .name = "per_pwrdm" },
	.cm_inst	= AM33XX_CM_PER_MOD,
	.prcm_partition	= AM33XX_PRM_PARTITION,
	.clkdm_offs	= AM33XX_CM_PER_L3_CLKSTCTRL_OFFSET,
	.clktrctrl_mask	= AM33XX_CLKTRCTRL_MASK,
	.flags		= (CLKDM_CAN_SWSUP | CLKDM_NO_AUTODEPS),
};

static struct clockdomain l4hs_am33xx_clkdm = {
	.name		= "l4hs_clkdm",
	.pwrdm		= { .name = "per_pwrdm" },
	.cm_inst	= AM33XX_CM_PER_MOD,
	.prcm_partition	= AM33XX_PRM_PARTITION,
	.clkdm_offs	= AM33XX_CM_PER_L4HS_CLKSTCTRL_OFFSET,
	.clktrctrl_mask	= AM33XX_CLKTRCTRL_MASK,
	.flags		= (CLKDM_CAN_SWSUP | CLKDM_NO_AUTODEPS),
};

static struct clockdomain ocpwp_l3_am33xx_clkdm = {
	.name		= "ocpwp_l3_clkdm",
	.pwrdm		= { .name = "per_pwrdm" },
	.cm_inst	= AM33XX_CM_PER_MOD,
	.prcm_partition	= AM33XX_PRM_PARTITION,
	.clkdm_offs	= AM33XX_CM_PER_OCPWP_L3_CLKSTCTRL_OFFSET,
	.clktrctrl_mask	= AM33XX_CLKTRCTRL_MASK,
	.flags		= (CLKDM_CAN_SWSUP | CLKDM_NO_AUTODEPS),
};

static struct clockdomain pruss_ocp_am33xx_clkdm = {
	.name		= "pruss_ocp_clkdm",
	.pwrdm		= { .name = "per_pwrdm" },
	.cm_inst	= AM33XX_CM_PER_MOD,
	.prcm_partition	= AM33XX_PRM_PARTITION,
	.clkdm_offs	= AM33XX_CM_PER_PRUSS_CLKSTCTRL_OFFSET,
	.clktrctrl_mask	= AM33XX_CLKTRCTRL_MASK,
	.flags		= (CLKDM_CAN_SWSUP | CLKDM_NO_AUTODEPS),
};

static struct clockdomain cpsw_125mhz_am33xx_clkdm = {
	.name		= "cpsw_125mhz_clkdm",
	.pwrdm		= { .name = "per_pwrdm" },
	.cm_inst	= AM33XX_CM_PER_MOD,
	.prcm_partition	= AM33XX_PRM_PARTITION,
	.clkdm_offs	= AM33XX_CM_PER_CPSW_CLKSTCTRL_OFFSET,
	.clktrctrl_mask	= AM33XX_CLKTRCTRL_MASK,
	.flags		= (CLKDM_CAN_SWSUP | CLKDM_NO_AUTODEPS),
};

static struct clockdomain lcdc_am33xx_clkdm = {
	.name		= "lcdc_clkdm",
	.pwrdm		= { .name = "per_pwrdm" },
	.cm_inst	= AM33XX_CM_PER_MOD,
	.prcm_partition	= AM33XX_PRM_PARTITION,
	.clkdm_offs	= AM33XX_CM_PER_LCDC_CLKSTCTRL_OFFSET,
	.clktrctrl_mask	= AM33XX_CLKTRCTRL_MASK,
	.flags		= (CLKDM_CAN_SWSUP | CLKDM_NO_AUTODEPS),
};

static struct clockdomain clk_24mhz_am33xx_clkdm = {
	.name		= "clk_24mhz_clkdm",
	.pwrdm		= { .name = "per_pwrdm" },
	.cm_inst	= AM33XX_CM_PER_MOD,
	.prcm_partition	= AM33XX_PRM_PARTITION,
	.clkdm_offs	= AM33XX_CM_PER_CLK_24MHZ_CLKSTCTRL_OFFSET,
	.clktrctrl_mask	= AM33XX_CLKTRCTRL_MASK,
	.flags		= (CLKDM_CAN_SWSUP | CLKDM_NO_AUTODEPS),
};

static struct clockdomain clk_25mhz_am33xx_clkdm = {
	.name		= "clk_25mhz_clkdm",
	.pwrdm		= { .name = "per_pwrdm" },
	.cm_inst	= AM33XX_CM_PER_MOD,
	.prcm_partition	= AM33XX_PRM_PARTITION,
	.clkdm_offs	= AM33XX_CM_PER_CLK_24MHZ_CLKSTCTRL_OFFSET,
	.clktrctrl_mask	= AM33XX_CLKTRCTRL_MASK,
	.flags		= (CLKDM_CAN_SWSUP | CLKDM_NO_AUTODEPS),
};

static struct clockdomain l4_wkup_am33xx_clkdm = {
	.name		= "l4_wkup_clkdm",
	.pwrdm		= { .name = "wkup_pwrdm" },
	.cm_inst	= AM33XX_CM_WKUP_MOD,
	.prcm_partition	= AM33XX_PRM_PARTITION,
	.clkdm_offs	= AM33XX_CM_WKUP_CLKSTCTRL_OFFSET,
	.clktrctrl_mask	= AM33XX_CLKTRCTRL_MASK,
	.flags		= (CLKDM_CAN_SWSUP | CLKDM_NO_AUTODEPS),
};

static struct clockdomain l3_aon_am33xx_clkdm = {
	.name		= "l3_aon_clkdm",
	.pwrdm		= { .name = "wkup_pwrdm" },
	.cm_inst	= AM33XX_CM_WKUP_MOD,
	.prcm_partition	= AM33XX_PRM_PARTITION,
	.clkdm_offs	= AM33XX_CM_L3_AON_CLKSTCTRL_OFFSET,
	.clktrctrl_mask	= AM33XX_CLKTRCTRL_MASK,
	.flags		= (CLKDM_CAN_SWSUP | CLKDM_NO_AUTODEPS),
};

static struct clockdomain l4_wkup_aon_am33xx_clkdm = {
	.name		= "l4_wkup_aon_clkdm",
	.pwrdm		= { .name = "wkup_pwrdm" },
	.cm_inst	= AM33XX_CM_WKUP_MOD,
	.prcm_partition	= AM33XX_PRM_PARTITION,
	.clkdm_offs	= AM33XX_CM_L4_WKUP_AON_CLKSTCTRL_OFFSET,
	.clktrctrl_mask	= AM33XX_CLKTRCTRL_MASK,
	.flags		= (CLKDM_CAN_SWSUP | CLKDM_NO_AUTODEPS),
};

static struct clockdomain mpu_am33xx_clkdm = {
	.name		= "mpu_clkdm",
	.pwrdm		= { .name = "mpu_pwrdm" },
	.cm_inst	= AM33XX_CM_MPU_MOD,
	.prcm_partition	= AM33XX_PRM_PARTITION,
	.clkdm_offs	= AM33XX_CM_MPU_CLKSTCTRL_OFFSET,
	.clktrctrl_mask	= AM33XX_CLKTRCTRL_MASK,
	.flags		= (CLKDM_CAN_SWSUP | CLKDM_NO_AUTODEPS),
};

static struct clockdomain l4_rtc_am33xx_clkdm = {
	.name		= "l4_rtc_clkdm",
	.pwrdm		= { .name = "rtc_pwrdm" },
	.cm_inst	= AM33XX_CM_RTC_MOD,
	.prcm_partition	= AM33XX_PRM_PARTITION,
	.clkdm_offs	= AM33XX_CM_RTC_CLKSTCTRL_OFFSET,
	.clktrctrl_mask	= AM33XX_CLKTRCTRL_MASK,
	.flags		= (CLKDM_CAN_SWSUP | CLKDM_NO_AUTODEPS),
};

static struct clockdomain gfx_l3_am33xx_clkdm = {
	.name		= "gfx_l3_clkdm",
	.pwrdm		= { .name = "gfx_pwrdm" },
	.cm_inst	= AM33XX_CM_GFX_MOD,
	.prcm_partition	= AM33XX_PRM_PARTITION,
	.clkdm_offs	= AM33XX_CM_GFX_L3_CLKSTCTRL_OFFSET,
	.clktrctrl_mask	= AM33XX_CLKTRCTRL_MASK,
	.flags		= (CLKDM_CAN_SWSUP | CLKDM_NO_AUTODEPS),
};

static struct clockdomain gfx_l4ls_gfx_am33xx_clkdm = {
	.name		= "gfx_l4ls_gfx_clkdm",
	.pwrdm		= { .name = "gfx_pwrdm" },
	.cm_inst	= AM33XX_CM_GFX_MOD,
	.prcm_partition	= AM33XX_PRM_PARTITION,
	.clkdm_offs	= AM33XX_CM_GFX_L4LS_GFX_CLKSTCTRL__1_OFFSET,
	.clktrctrl_mask	= AM33XX_CLKTRCTRL_MASK,
	.flags		= (CLKDM_CAN_SWSUP | CLKDM_NO_AUTODEPS),
};

static struct clockdomain l4_cefuse_am33xx_clkdm = {
	.name		= "l4_cefuse_clkdm",
	.pwrdm		= { .name = "cefuse_pwrdm" },
	.cm_inst	= AM33XX_CM_CEFUSE_MOD,
	.prcm_partition	= AM33XX_PRM_PARTITION,
	.clkdm_offs	= AM33XX_CM_CEFUSE_CLKSTCTRL_OFFSET,
	.clktrctrl_mask	= AM33XX_CLKTRCTRL_MASK,
	.flags		= (CLKDM_CAN_SWSUP | CLKDM_NO_AUTODEPS),
};

static struct clockdomain *clockdomains_am33xx[] __initdata = {
	&l4ls_am33xx_clkdm,
	&l3s_am33xx_clkdm,
	&l4fw_am33xx_clkdm,
	&l3_am33xx_clkdm,
	&l4hs_am33xx_clkdm,
	&ocpwp_l3_am33xx_clkdm,
	&pruss_ocp_am33xx_clkdm,
	&cpsw_125mhz_am33xx_clkdm,
	&lcdc_am33xx_clkdm,
	&clk_24mhz_am33xx_clkdm,
	&clk_25mhz_am33xx_clkdm,
	&l4_wkup_am33xx_clkdm,
	&l3_aon_am33xx_clkdm,
	&l4_wkup_aon_am33xx_clkdm,
	&mpu_am33xx_clkdm,
	&l4_rtc_am33xx_clkdm,
	&gfx_l3_am33xx_clkdm,
	&gfx_l4ls_gfx_am33xx_clkdm,
	&l4_cefuse_am33xx_clkdm,
	NULL,
};

void __init am33xx_clockdomains_init(void)
{
	clkdm_register_platform_funcs(&am33xx_clkdm_operations);
	clkdm_register_clkdms(clockdomains_am33xx);
	clkdm_complete_init();
}
