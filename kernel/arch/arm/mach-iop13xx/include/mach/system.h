
#include <mach/iop13xx.h>
static inline void arch_idle(void)
{
	cpu_do_idle();
}

static inline void arch_reset(char mode, const char *cmd)
{
	/*
	 * Reset the internal bus (warning both cores are reset)
	 */
	write_wdtcr(IOP_WDTCR_EN_ARM);
	write_wdtcr(IOP_WDTCR_EN);
	write_wdtsr(IOP13XX_WDTSR_WRITE_EN | IOP13XX_WDTCR_IB_RESET);
	write_wdtcr(0x1000);

	for(;;);
}
