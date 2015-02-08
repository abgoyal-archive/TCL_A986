

#ifndef _XTENSA_HARDIRQ_H
#define _XTENSA_HARDIRQ_H

void ack_bad_irq(unsigned int irq);
#define ack_bad_irq ack_bad_irq

#include <asm-generic/hardirq.h>

#endif	/* _XTENSA_HARDIRQ_H */
