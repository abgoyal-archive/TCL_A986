
#ifndef _ASM_IA64_SN_PCI_PCIBUS_PROVIDER_H
#define _ASM_IA64_SN_PCI_PCIBUS_PROVIDER_H


#define PCIIO_ASIC_TYPE_UNKNOWN	0
#define PCIIO_ASIC_TYPE_PPB	1
#define PCIIO_ASIC_TYPE_PIC	2
#define PCIIO_ASIC_TYPE_TIOCP	3
#define PCIIO_ASIC_TYPE_TIOCA	4
#define PCIIO_ASIC_TYPE_TIOCE	5

#define PCIIO_ASIC_MAX_TYPES	6


struct pcibus_bussoft {
	u32		bs_asic_type;	/* chipset type */
	u32		bs_xid;		/* xwidget id */
	u32		bs_persist_busnum; /* Persistent Bus Number */
	u32		bs_persist_segment; /* Segment Number */
	u64		bs_legacy_io;	/* legacy io pio addr */
	u64		bs_legacy_mem;	/* legacy mem pio addr */
	u64		bs_base;	/* widget base */
	struct xwidget_info	*bs_xwidget_info;
};

struct pci_controller;

struct sn_pcibus_provider {
	dma_addr_t	(*dma_map)(struct pci_dev *, unsigned long, size_t, int flags);
	dma_addr_t	(*dma_map_consistent)(struct pci_dev *, unsigned long, size_t, int flags);
	void		(*dma_unmap)(struct pci_dev *, dma_addr_t, int);
	void *		(*bus_fixup)(struct pcibus_bussoft *, struct pci_controller *);
 	void		(*force_interrupt)(struct sn_irq_info *);
 	void		(*target_interrupt)(struct sn_irq_info *);
};


#define SN_DMA_ADDRTYPE(x)	((x) & 0xf)
#define     SN_DMA_ADDR_PHYS	1	/* address is an xio address. */
#define     SN_DMA_ADDR_XIO	2	/* address is phys memory */
#define SN_DMA_MSI		0x10	/* Bus address is to be used for MSI */

extern struct sn_pcibus_provider *sn_pci_provider[];
#endif				/* _ASM_IA64_SN_PCI_PCIBUS_PROVIDER_H */
