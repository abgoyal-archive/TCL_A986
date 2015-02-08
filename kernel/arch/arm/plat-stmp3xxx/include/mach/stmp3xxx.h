

#ifndef __ASM_PLAT_STMP3XXX_H
#define __ASM_PLAT_STMP3XXX_H

#include <linux/irq.h>

extern struct sys_timer stmp3xxx_timer;

void stmp3xxx_init_irq(struct irq_chip *chip);
void stmp3xxx_init(void);
int stmp3xxx_reset_block(void __iomem *hwreg, int just_enable);
extern struct platform_device stmp3xxx_dbguart,
			      stmp3xxx_appuart,
			      stmp3xxx_watchdog,
			      stmp3xxx_touchscreen,
			      stmp3xxx_keyboard,
			      stmp3xxx_gpmi,
			      stmp3xxx_mmc,
			      stmp3xxx_udc,
			      stmp3xxx_ehci,
			      stmp3xxx_rtc,
			      stmp3xxx_spi1,
			      stmp3xxx_spi2,
			      stmp3xxx_backlight,
			      stmp3xxx_rotdec,
			      stmp3xxx_dcp,
			      stmp3xxx_dcp_bootstream,
			      stmp3xxx_persistent,
			      stmp3xxx_framebuffer,
			      stmp3xxx_battery;
int stmp3xxx_ssp1_device_register(void);
int stmp3xxx_ssp2_device_register(void);

struct pin_group;
void stmp3xxx_release_pin_group(struct pin_group *pin_group, const char *label);
int stmp3xxx_request_pin_group(struct pin_group *pin_group, const char *label);

#endif /* __ASM_PLAT_STMP3XXX_H */
