

#ifndef __ASM_DEC_IOASIC_H
#define __ASM_DEC_IOASIC_H

#include <linux/spinlock.h>
#include <linux/types.h>

extern spinlock_t ioasic_ssr_lock;

extern volatile u32 *ioasic_base;

static inline void ioasic_write(unsigned int reg, u32 v)
{
	ioasic_base[reg / 4] = v;
}

static inline u32 ioasic_read(unsigned int reg)
{
	return ioasic_base[reg / 4];
}

extern void init_ioasic_irqs(int base);

extern void dec_ioasic_clocksource_init(void);

#endif /* __ASM_DEC_IOASIC_H */
