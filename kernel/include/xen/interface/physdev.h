

#ifndef __XEN_PUBLIC_PHYSDEV_H__
#define __XEN_PUBLIC_PHYSDEV_H__


#define PHYSDEVOP_eoi			12
struct physdev_eoi {
	/* IN */
	uint32_t irq;
};

#define PHYSDEVOP_irq_status_query	 5
struct physdev_irq_status_query {
	/* IN */
	uint32_t irq;
	/* OUT */
	uint32_t flags; /* XENIRQSTAT_* */
};

/* Need to call PHYSDEVOP_eoi when the IRQ has been serviced? */
#define _XENIRQSTAT_needs_eoi	(0)
#define	 XENIRQSTAT_needs_eoi	(1U<<_XENIRQSTAT_needs_eoi)

/* IRQ shared by multiple guests? */
#define _XENIRQSTAT_shared	(1)
#define	 XENIRQSTAT_shared	(1U<<_XENIRQSTAT_shared)

#define PHYSDEVOP_set_iopl		 6
struct physdev_set_iopl {
	/* IN */
	uint32_t iopl;
};

#define PHYSDEVOP_set_iobitmap		 7
struct physdev_set_iobitmap {
	/* IN */
	uint8_t * bitmap;
	uint32_t nr_ports;
};

#define PHYSDEVOP_apic_read		 8
#define PHYSDEVOP_apic_write		 9
struct physdev_apic {
	/* IN */
	unsigned long apic_physbase;
	uint32_t reg;
	/* IN or OUT */
	uint32_t value;
};

#define PHYSDEVOP_alloc_irq_vector	10
#define PHYSDEVOP_free_irq_vector	11
struct physdev_irq {
	/* IN */
	uint32_t irq;
	/* IN or OUT */
	uint32_t vector;
};

struct physdev_op {
	uint32_t cmd;
	union {
		struct physdev_irq_status_query	     irq_status_query;
		struct physdev_set_iopl		     set_iopl;
		struct physdev_set_iobitmap	     set_iobitmap;
		struct physdev_apic		     apic_op;
		struct physdev_irq		     irq_op;
	} u;
};

#define PHYSDEVOP_IRQ_UNMASK_NOTIFY	 4

#define PHYSDEVOP_IRQ_STATUS_QUERY	 PHYSDEVOP_irq_status_query
#define PHYSDEVOP_SET_IOPL		 PHYSDEVOP_set_iopl
#define PHYSDEVOP_SET_IOBITMAP		 PHYSDEVOP_set_iobitmap
#define PHYSDEVOP_APIC_READ		 PHYSDEVOP_apic_read
#define PHYSDEVOP_APIC_WRITE		 PHYSDEVOP_apic_write
#define PHYSDEVOP_ASSIGN_VECTOR		 PHYSDEVOP_alloc_irq_vector
#define PHYSDEVOP_FREE_VECTOR		 PHYSDEVOP_free_irq_vector
#define PHYSDEVOP_IRQ_NEEDS_UNMASK_NOTIFY XENIRQSTAT_needs_eoi
#define PHYSDEVOP_IRQ_SHARED		 XENIRQSTAT_shared

#endif /* __XEN_PUBLIC_PHYSDEV_H__ */
