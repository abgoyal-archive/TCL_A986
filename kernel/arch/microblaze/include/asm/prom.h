

#include <linux/of.h>	/* linux/of.h gets to determine #include ordering */

#ifndef _ASM_MICROBLAZE_PROM_H
#define _ASM_MICROBLAZE_PROM_H
#ifdef __KERNEL__
#ifndef __ASSEMBLY__

#include <linux/types.h>
#include <linux/of_fdt.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <asm/irq.h>
#include <asm/atomic.h>

#define HAVE_ARCH_DEVTREE_FIXUPS

/* Other Prototypes */
extern int early_uartlite_console(void);

#ifdef CONFIG_PCI
struct pci_bus;
struct pci_dev;
extern int pci_device_from_OF_node(struct device_node *node,
					u8 *bus, u8 *devfn);
extern struct device_node *pci_busdev_to_OF_node(struct pci_bus *bus,
							int devfn);
extern struct device_node *pci_device_to_OF_node(struct pci_dev *dev);
extern void pci_create_OF_bus_map(void);
#endif


extern u64 of_translate_address(struct device_node *np, const u32 *addr);

extern const u32 *of_get_address(struct device_node *dev, int index,
			u64 *size, unsigned int *flags);
extern const u32 *of_get_pci_address(struct device_node *dev, int bar_no,
			u64 *size, unsigned int *flags);

extern int of_address_to_resource(struct device_node *dev, int index,
				struct resource *r);
extern int of_pci_address_to_resource(struct device_node *dev, int bar,
				struct resource *r);

void of_parse_dma_window(struct device_node *dn, const void *dma_window_prop,
		unsigned long *busno, unsigned long *phys, unsigned long *size);

extern void kdump_move_device_tree(void);

/* CPU OF node matching */
struct device_node *of_get_cpu_node(int cpu, unsigned int *thread);

/* Get the MAC address */
extern const void *of_get_mac_address(struct device_node *np);



#define OF_MAX_IRQ_SPEC		4 /* We handle specifiers of at most 4 cells */

struct of_irq {
	struct device_node *controller; /* Interrupt controller node */
	u32 size; /* Specifier size */
	u32 specifier[OF_MAX_IRQ_SPEC]; /* Specifier copy */
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

#endif /* __ASSEMBLY__ */
#endif /* __KERNEL__ */
#endif /* _ASM_MICROBLAZE_PROM_H */
