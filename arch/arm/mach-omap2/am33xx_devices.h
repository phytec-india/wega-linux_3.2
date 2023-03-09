#ifndef __ARCH_ARM_MACH_OMAP2_AM335X_DEVICES_H
#define __ARCH_ARM_MACH_OMAP2_AM335X_DEVICES_H

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

extern void am335x_nand_init(struct gpmc_timings *am335x_nand_timings);
extern void am335x_spi0_init(
			struct spi_board_info const *am335x_spi0_slave_info,
			unsigned size);
extern void am335x_mmc0_init(int gpio_cd, int gpio_wp);
extern void am33xx_cpuidle_init(void);
extern void am33xx_clkout1_enable(void);
extern void am33xx_tps65910_init(int i2c_busnum, int gpio_irq);
extern void am33xx_rmii1_mii2_init(unsigned char *phy_id0,
						unsigned char *phy_id1);
extern void am33xx_usb0_otg_usb1_host_init(void);

extern void am33xx_d_can_init(unsigned int instance);

extern void am33xx_ecap2_init(unsigned int dft_brightness);
#endif
