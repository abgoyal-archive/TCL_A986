
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/sizes.h>
#include <mach/memory.h>
#include <cfg_global.h>
#include <mach/csp/mm_io.h>

#define RAM_START               PHYS_OFFSET

#define RAM_SIZE                (CFG_GLOBAL_RAM_SIZE-CFG_GLOBAL_RAM_SIZE_RESERVED)
#define RAM_BASE                PAGE_OFFSET

#define pcibios_assign_all_busses()	1

/* Macros to make managing spinlocks a bit more controlled in terms of naming. */
/* See reg_gpio.h, reg_irq.h, arch.c, gpio.c for example usage. */
#if defined(__KERNEL__)
#define HW_DECLARE_SPINLOCK(name)  DEFINE_SPINLOCK(bcmring_##name##_reg_lock);
#define HW_EXTERN_SPINLOCK(name)   extern spinlock_t bcmring_##name##_reg_lock;
#define HW_IRQ_SAVE(name, val)     spin_lock_irqsave(&bcmring_##name##_reg_lock, (val))
#define HW_IRQ_RESTORE(name, val)  spin_unlock_irqrestore(&bcmring_##name##_reg_lock, (val))
#else
#define HW_DECLARE_SPINLOCK(name)
#define HW_EXTERN_SPINLOCK(name)
#define HW_IRQ_SAVE(name, val)     {(void)(name); (void)(val); }
#define HW_IRQ_RESTORE(name, val)  {(void)(name); (void)(val); }
#endif

#ifndef HW_IO_PHYS_TO_VIRT
#define HW_IO_PHYS_TO_VIRT MM_IO_PHYS_TO_VIRT
#endif
#define HW_IO_VIRT_TO_PHYS MM_IO_VIRT_TO_PHYS

#endif
