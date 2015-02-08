
#include <linux/of.h>	/* linux/of.h gets to determine #include ordering */
#ifndef _POWERPC_PROM_H
#define _POWERPC_PROM_H
#ifdef __KERNEL__

#include <linux/types.h>
#include <linux/of_fdt.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <asm/irq.h>
#include <asm/atomic.h>

#define HAVE_ARCH_DEVTREE_FIXUPS

#ifdef CONFIG_PPC32
struct pci_bus;
struct pci_dev;
extern int pci_device_from_OF_node(struct device_node *node,
				   u8* bus, u8* devfn);
extern struct device_node* pci_busdev_to_OF_node(struct pci_bus *, int);
extern struct device_node* pci_device_to_OF_node(struct pci_dev *);
extern void pci_create_OF_bus_map(void);
#endif


extern u64 of_translate_address(struct device_node *np, const u32 *addr);

/* Translate a DMA address from device space to CPU space */
extern u64 of_translate_dma_address(struct device_node *dev,
				    const u32 *in_addr);

extern const u32 *of_get_address(struct device_node *dev, int index,
			   u64 *size, unsigned int *flags);
#ifdef CONFIG_PCI
extern const u32 *of_get_pci_address(struct device_node *dev, int bar_no,
			       u64 *size, unsigned int *flags);
#else
static inline const u32 *of_get_pci_address(struct device_node *dev,
		int bar_no, u64 *size, unsigned int *flags)
{
	return NULL;
}
#endif /* CONFIG_PCI */

extern int of_address_to_resource(struct device_node *dev, int index,
				  struct resource *r);
#ifdef CONFIG_PCI
extern int of_pci_address_to_resource(struct device_node *dev, int bar,
				      struct resource *r);
#else
static inline int of_pci_address_to_resource(struct device_node *dev, int bar,
		struct resource *r)
{
	return -ENOSYS;
}
#endif /* CONFIG_PCI */

void of_parse_dma_window(struct device_node *dn, const void *dma_window_prop,
		unsigned long *busno, unsigned long *phys, unsigned long *size);

extern void kdump_move_device_tree(void);

/* CPU OF node matching */
struct device_node *of_get_cpu_node(int cpu, unsigned int *thread);

/* cache lookup */
struct device_node *of_find_next_cache_node(struct device_node *np);

/* Get the MAC address */
extern const void *of_get_mac_address(struct device_node *np);



#define OF_MAX_IRQ_SPEC		 4 /* We handle specifiers of at most 4 cells */

struct of_irq {
	struct device_node *controller;	/* Interrupt controller node */
	u32 size;			/* Specifier size */
	u32 specifier[OF_MAX_IRQ_SPEC];	/* Specifier copy */
};

#define OF_IMAP_OLDWORLD_MAC	0x00000001
#define OF_IMAP_NO_PHANDLE	0x00000002

extern void of_irq_map_init(unsigned int flags);


extern int of_irq_map_raw(struct device_node *parent, const u32 *intspec,
			  u32 ointsize, const u32 *addr,
			  struct of_irq *out_irq);


extern int of_irq_map_one(struct device_node *device, int index,
			  struct of_irq *out_irq);

struct pci_dev;
extern int of_irq_map_pci(struct pci_dev *pdev, struct of_irq *out_irq);

extern int of_irq_to_resource(struct device_node *dev, int index,
			struct resource *r);

extern void __iomem *of_iomap(struct device_node *device, int index);

#endif /* __KERNEL__ */
#endif /* _POWERPC_PROM_H */
