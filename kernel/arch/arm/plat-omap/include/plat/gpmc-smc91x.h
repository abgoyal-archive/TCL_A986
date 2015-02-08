

#ifndef __ASM_ARCH_OMAP_GPMC_SMC91X_H__

#define GPMC_TIMINGS_SMC91C96	(1 << 4)
#define GPMC_MUX_ADD_DATA	(1 << 5) /* GPMC_CONFIG1_MUXADDDATA */
#define GPMC_READ_MON		(1 << 6) /* GPMC_CONFIG1_WAIT_READ_MON */
#define GPMC_WRITE_MON		(1 << 7) /* GPMC_CONFIG1_WAIT_WRITE_MON */

struct omap_smc91x_platform_data {
	int	cs;
	int	gpio_irq;
	int	gpio_pwrdwn;
	int	gpio_reset;
	int	wait_pin;	/* Optional GPMC_CONFIG1_WAITPINSELECT */
	u32	flags;
	int	(*retime)(void);
};

#if defined(CONFIG_SMC91X) || \
	defined(CONFIG_SMC91X_MODULE)

extern void gpmc_smc91x_init(struct omap_smc91x_platform_data *d);

#else

#define board_smc91x_data	NULL

static inline void gpmc_smc91x_init(struct omap_smc91x_platform_data *d)
{
}

#endif
#endif
