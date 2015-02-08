

#ifndef __IRQ_H
#define __IRQ_H

#include "lapic.h"

static inline int irqchip_in_kernel(struct kvm *kvm)
{
	return 1;
}

#endif
