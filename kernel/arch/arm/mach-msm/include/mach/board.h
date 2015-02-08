

#ifndef __ASM_ARCH_MSM_BOARD_H
#define __ASM_ARCH_MSM_BOARD_H

#include <linux/types.h>

/* platform device data structures */

struct msm_acpu_clock_platform_data
{
	uint32_t acpu_switch_time_us;
	uint32_t max_speed_delta_khz;
	uint32_t vdd_switch_time_us;
	unsigned long power_collapse_khz;
	unsigned long wait_for_irq_khz;
};

struct clk;

/* common init routines for use by arch/arm/mach-msm/board-*.c */

void __init msm_add_devices(void);
void __init msm_map_common_io(void);
void __init msm_init_irq(void);
void __init msm_init_gpio(void);
void __init msm_clock_init(struct clk *clock_tbl, unsigned num_clocks);
void __init msm_acpu_clock_init(struct msm_acpu_clock_platform_data *);

#endif
