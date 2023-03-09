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

#endif
