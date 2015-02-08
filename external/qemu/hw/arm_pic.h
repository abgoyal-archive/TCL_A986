

#ifndef ARM_INTERRUPT_H
#define ARM_INTERRUPT_H 1

#include "irq.h"

/* The CPU is also modeled as an interrupt controller.  */
#define ARM_PIC_CPU_IRQ 0
#define ARM_PIC_CPU_FIQ 1
qemu_irq *arm_pic_init_cpu(CPUState *env);

#endif /* !ARM_INTERRUPT_H */

