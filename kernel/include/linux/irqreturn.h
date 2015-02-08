
#ifndef _LINUX_IRQRETURN_H
#define _LINUX_IRQRETURN_H

enum irqreturn {
	IRQ_NONE,
	IRQ_HANDLED,
	IRQ_WAKE_THREAD,
};

typedef enum irqreturn irqreturn_t;
#define IRQ_RETVAL(x)	((x) != IRQ_NONE)

#endif
