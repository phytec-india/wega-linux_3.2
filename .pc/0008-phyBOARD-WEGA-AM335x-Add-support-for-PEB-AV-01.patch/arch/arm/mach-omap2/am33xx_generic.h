#ifndef __ARCH_ARM_MACH_OMAP2_AM335X_GENERIC_H
#define __ARCH_ARM_MACH_OMAP2_AM335X_GENERIC_H

/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

extern void __iomem *am33xx_emif_base;
extern void __iomem * __init am33xx_get_mem_ctlr(void);
extern void __iomem *am33xx_get_ram_base(void);

extern void __iomem *am33xx_gpio0_base;
extern void __iomem *am33xx_get_gpio0_base(void);

struct pinmux_config {
	const char *string_name; /* signal name format */
	int val; /* Options for the mux register value */
};

void setup_pin_mux(struct pinmux_config *pin_mux);

void am335x_internal_rtc_init(void);

#endif
