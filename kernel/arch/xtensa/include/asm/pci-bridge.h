

#ifndef _XTENSA_PCI_BRIDGE_H
#define _XTENSA_PCI_BRIDGE_H

#ifdef __KERNEL__

struct device_node;
struct pci_controller;


extern int pciauto_bus_scan(struct pci_controller *, int);

struct pci_space {
	unsigned long start;
	unsigned long end;
	unsigned long base;
};


struct pci_controller {
	int index;			/* used for pci_controller_num */
	struct pci_controller *next;
        struct pci_bus *bus;
	void *arch_data;

	int first_busno;
	int last_busno;

	struct pci_ops *ops;
	volatile unsigned int *cfg_addr;
	volatile unsigned char *cfg_data;

	/* Currently, we limit ourselves to 1 IO range and 3 mem
	 * ranges since the common pci_bus structure can't handle more
	 */
	struct resource	io_resource;
	struct resource mem_resources[3];
	int mem_resource_count;

	/* Host bridge I/O and Memory space
	 * Used for BAR placement algorithms
	 */
	struct pci_space io_space;
	struct pci_space mem_space;

	/* Return the interrupt number fo a device. */
	int (*map_irq)(struct pci_dev*, u8, u8);

};

static inline void pcibios_init_resource(struct resource *res,
		unsigned long start, unsigned long end, int flags, char *name)
{
	res->start = start;
	res->end = end;
	res->flags = flags;
	res->name = name;
	res->parent = NULL;
	res->sibling = NULL;
	res->child = NULL;
}


/* These are used for config access before all the PCI probing has been done. */
int early_read_config_byte(struct pci_controller*, int, int, int, u8*);
int early_read_config_word(struct pci_controller*, int, int, int, u16*);
int early_read_config_dword(struct pci_controller*, int, int, int, u32*);
int early_write_config_byte(struct pci_controller*, int, int, int, u8);
int early_write_config_word(struct pci_controller*, int, int, int, u16);
int early_write_config_dword(struct pci_controller*, int, int, int, u32);

#endif	/* __KERNEL__ */
#endif	/* _XTENSA_PCI_BRIDGE_H */
