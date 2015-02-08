
#include <asm/mach-types.h>
#include <asm/hardware/iop3xx.h>
#include <mach/n2100.h>

static inline void arch_idle(void)
{
	cpu_do_idle();
}

static inline void arch_reset(char mode, const char *cmd)
{
	local_irq_disable();

	if (machine_is_n2100()) {
		gpio_line_set(N2100_HARDWARE_RESET, GPIO_LOW);
		gpio_line_config(N2100_HARDWARE_RESET, GPIO_OUT);
		while (1)
			;
	}

	*IOP3XX_PCSR = 0x30;

	/* Jump into ROM at address 0 */
	cpu_reset(0);
}
