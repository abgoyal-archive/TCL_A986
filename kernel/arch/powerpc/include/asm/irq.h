
#ifdef __KERNEL__
#ifndef _ASM_POWERPC_IRQ_H
#define _ASM_POWERPC_IRQ_H


#include <linux/threads.h>
#include <linux/list.h>
#include <linux/radix-tree.h>

#include <asm/types.h>
#include <asm/atomic.h>


/* Define a way to iterate across irqs. */
#define for_each_irq(i) \
	for ((i) = 0; (i) < NR_IRQS; ++(i))

extern atomic_t ppc_n_lost_interrupts;

/* This number is used when no interrupt has been assigned */
#define NO_IRQ			(0)

#define NO_IRQ_IGNORE		((unsigned int)-1)

/* Total number of virq in the platform */
#define NR_IRQS		CONFIG_NR_IRQS

/* Number of irqs reserved for the legacy controller */
#define NUM_ISA_INTERRUPTS	16

/* Same thing, used by the generic IRQ code */
#define NR_IRQS_LEGACY		NUM_ISA_INTERRUPTS

typedef unsigned long irq_hw_number_t;

struct irq_host;
struct radix_tree_root;

struct irq_host_ops {
	/* Match an interrupt controller device node to a host, returns
	 * 1 on a match
	 */
	int (*match)(struct irq_host *h, struct device_node *node);

	/* Create or update a mapping between a virtual irq number and a hw
	 * irq number. This is called only once for a given mapping.
	 */
	int (*map)(struct irq_host *h, unsigned int virq, irq_hw_number_t hw);

	/* Dispose of such a mapping */
	void (*unmap)(struct irq_host *h, unsigned int virq);

	/* Update of such a mapping  */
	void (*remap)(struct irq_host *h, unsigned int virq, irq_hw_number_t hw);

	/* Translate device-tree interrupt specifier from raw format coming
	 * from the firmware to a irq_hw_number_t (interrupt line number) and
	 * type (sense) that can be passed to set_irq_type(). In the absence
	 * of this callback, irq_create_of_mapping() and irq_of_parse_and_map()
	 * will return the hw number in the first cell and IRQ_TYPE_NONE for
	 * the type (which amount to keeping whatever default value the
	 * interrupt controller has for that line)
	 */
	int (*xlate)(struct irq_host *h, struct device_node *ctrler,
		     const u32 *intspec, unsigned int intsize,
		     irq_hw_number_t *out_hwirq, unsigned int *out_type);
};

struct irq_host {
	struct list_head	link;

	/* type of reverse mapping technique */
	unsigned int		revmap_type;
#define IRQ_HOST_MAP_LEGACY     0 /* legacy 8259, gets irqs 1..15 */
#define IRQ_HOST_MAP_NOMAP	1 /* no fast reverse mapping */
#define IRQ_HOST_MAP_LINEAR	2 /* linear map of interrupts */
#define IRQ_HOST_MAP_TREE	3 /* radix tree */
	union {
		struct {
			unsigned int size;
			unsigned int *revmap;
		} linear;
		struct radix_tree_root tree;
	} revmap_data;
	struct irq_host_ops	*ops;
	void			*host_data;
	irq_hw_number_t		inval_irq;

	/* Optional device node pointer */
	struct device_node	*of_node;
};

struct irq_map_entry {
	irq_hw_number_t	hwirq;
	struct irq_host	*host;
};

extern struct irq_map_entry irq_map[NR_IRQS];

extern irq_hw_number_t virq_to_hw(unsigned int virq);

extern struct irq_host *irq_alloc_host(struct device_node *of_node,
				       unsigned int revmap_type,
				       unsigned int revmap_arg,
				       struct irq_host_ops *ops,
				       irq_hw_number_t inval_irq);


extern struct irq_host *irq_find_host(struct device_node *node);


extern void irq_set_default_host(struct irq_host *host);


extern void irq_set_virq_count(unsigned int count);


extern unsigned int irq_create_mapping(struct irq_host *host,
				       irq_hw_number_t hwirq);


extern void irq_dispose_mapping(unsigned int virq);

extern unsigned int irq_find_mapping(struct irq_host *host,
				     irq_hw_number_t hwirq);

extern unsigned int irq_create_direct_mapping(struct irq_host *host);

extern void irq_radix_revmap_insert(struct irq_host *host, unsigned int virq,
				    irq_hw_number_t hwirq);

extern unsigned int irq_radix_revmap_lookup(struct irq_host *host,
					    irq_hw_number_t hwirq);


extern unsigned int irq_linear_revmap(struct irq_host *host,
				      irq_hw_number_t hwirq);



extern unsigned int irq_alloc_virt(struct irq_host *host,
				   unsigned int count,
				   unsigned int hint);

extern void irq_free_virt(unsigned int virq, unsigned int count);


/* -- OF helpers -- */

extern unsigned int irq_create_of_mapping(struct device_node *controller,
					  const u32 *intspec, unsigned int intsize);

extern unsigned int irq_of_parse_and_map(struct device_node *dev, int index);

/* -- End OF helpers -- */

extern void irq_early_init(void);

static __inline__ int irq_canonicalize(int irq)
{
	return irq;
}

extern int distribute_irqs;

struct irqaction;
struct pt_regs;

#define __ARCH_HAS_DO_SOFTIRQ

#if defined(CONFIG_BOOKE) || defined(CONFIG_40x)
extern struct thread_info *critirq_ctx[NR_CPUS];
extern struct thread_info *dbgirq_ctx[NR_CPUS];
extern struct thread_info *mcheckirq_ctx[NR_CPUS];
extern void exc_lvl_ctx_init(void);
#else
#define exc_lvl_ctx_init()
#endif

extern struct thread_info *hardirq_ctx[NR_CPUS];
extern struct thread_info *softirq_ctx[NR_CPUS];

extern void irq_ctx_init(void);
extern void call_do_softirq(struct thread_info *tp);
extern int call_handle_irq(int irq, void *p1,
			   struct thread_info *tp, void *func);
extern void do_IRQ(struct pt_regs *regs);

#endif /* _ASM_IRQ_H */
#endif /* __KERNEL__ */
