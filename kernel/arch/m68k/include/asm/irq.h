
#ifndef _M68K_IRQ_H_
#define _M68K_IRQ_H_

#if defined(CONFIG_COLDFIRE)
#define NR_IRQS 256
#elif defined(CONFIG_VME) || defined(CONFIG_SUN3) || defined(CONFIG_SUN3X)
#define NR_IRQS 200
#elif defined(CONFIG_ATARI) || defined(CONFIG_MAC)
#define NR_IRQS 72
#elif defined(CONFIG_Q40)
#define NR_IRQS	43
#elif defined(CONFIG_AMIGA) || !defined(CONFIG_MMU)
#define NR_IRQS	32
#elif defined(CONFIG_APOLLO)
#define NR_IRQS	24
#elif defined(CONFIG_HP300)
#define NR_IRQS	8
#else
#define NR_IRQS	0
#endif

#ifdef CONFIG_MMU

#include <linux/linkage.h>
#include <linux/hardirq.h>
#include <linux/irqreturn.h>
#include <linux/spinlock_types.h>

#if (1 << HARDIRQ_BITS) < NR_IRQS
# error HARDIRQ_BITS is too low!
#endif


#define IRQ_SPURIOUS	0

#define IRQ_AUTO_1	1	/* level 1 interrupt */
#define IRQ_AUTO_2	2	/* level 2 interrupt */
#define IRQ_AUTO_3	3	/* level 3 interrupt */
#define IRQ_AUTO_4	4	/* level 4 interrupt */
#define IRQ_AUTO_5	5	/* level 5 interrupt */
#define IRQ_AUTO_6	6	/* level 6 interrupt */
#define IRQ_AUTO_7	7	/* level 7 interrupt (non-maskable) */

#define IRQ_USER	8

extern unsigned int irq_canonicalize(unsigned int irq);

struct pt_regs;

#ifndef MACH_AMIGA_ONLY
#define IRQ_FLG_LOCK	(0x0001)	/* handler is not replaceable	*/
#define IRQ_FLG_REPLACE	(0x0002)	/* replace existing handler	*/
#define IRQ_FLG_FAST	(0x0004)
#define IRQ_FLG_SLOW	(0x0008)
#define IRQ_FLG_STD	(0x8000)	/* internally used		*/
#endif

typedef struct irq_node {
	irqreturn_t	(*handler)(int, void *);
	void		*dev_id;
	struct irq_node *next;
	unsigned long	flags;
	const char	*devname;
} irq_node_t;

struct irq_handler {
	int		(*handler)(int, void *);
	unsigned long	flags;
	void		*dev_id;
	const char	*devname;
};

struct irq_controller {
	const char *name;
	spinlock_t lock;
	int (*startup)(unsigned int irq);
	void (*shutdown)(unsigned int irq);
	void (*enable)(unsigned int irq);
	void (*disable)(unsigned int irq);
};

extern int m68k_irq_startup(unsigned int);
extern void m68k_irq_shutdown(unsigned int);

extern irq_node_t *new_irq_node(void);

extern void m68k_setup_auto_interrupt(void (*handler)(unsigned int, struct pt_regs *));
extern void m68k_setup_user_interrupt(unsigned int vec, unsigned int cnt,
				      void (*handler)(unsigned int, struct pt_regs *));
extern void m68k_setup_irq_controller(struct irq_controller *, unsigned int, unsigned int);

asmlinkage void m68k_handle_int(unsigned int);
asmlinkage void __m68k_handle_int(unsigned int, struct pt_regs *);

#else
#define irq_canonicalize(irq)  (irq)
#endif /* CONFIG_MMU */

#endif /* _M68K_IRQ_H_ */
