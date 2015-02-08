
#ifndef __ASM_AVR32_IRQFLAGS_H
#define __ASM_AVR32_IRQFLAGS_H

#include <asm/sysreg.h>

static inline unsigned long __raw_local_save_flags(void)
{
	return sysreg_read(SR);
}

#define raw_local_save_flags(x)					\
	do { (x) = __raw_local_save_flags(); } while (0)

static inline void raw_local_irq_restore(unsigned long flags)
{
	sysreg_write(SR, flags);
	asm volatile("" : : : "memory", "cc");
}

static inline void raw_local_irq_disable(void)
{
	asm volatile("ssrf %0" : : "n"(SYSREG_GM_OFFSET) : "memory");
}

static inline void raw_local_irq_enable(void)
{
	asm volatile("csrf %0" : : "n"(SYSREG_GM_OFFSET) : "memory");
}

static inline int raw_irqs_disabled_flags(unsigned long flags)
{
	return (flags & SYSREG_BIT(GM)) != 0;
}

static inline int raw_irqs_disabled(void)
{
	unsigned long flags = __raw_local_save_flags();

	return raw_irqs_disabled_flags(flags);
}

static inline unsigned long __raw_local_irq_save(void)
{
	unsigned long flags = __raw_local_save_flags();

	raw_local_irq_disable();

	return flags;
}

#define raw_local_irq_save(flags)				\
	do { (flags) = __raw_local_irq_save(); } while (0)

#endif /* __ASM_AVR32_IRQFLAGS_H */
