
#ifndef _PLATFORMS_ISERIES_PCI_H
#define _PLATFORMS_ISERIES_PCI_H



#define ISERIES_PCI_AGENTID(idsel, func)	\
	(((idsel & 0x0F) << 4) | (func & 0x07))
#define ISERIES_ENCODE_DEVICE(agentid)		\
	((0x10) | ((agentid & 0x20) >> 2) | (agentid & 0x07))

#define ISERIES_GET_DEVICE_FROM_SUBBUS(subbus)		((subbus >> 5) & 0x7)
#define ISERIES_GET_FUNCTION_FROM_SUBBUS(subbus)	((subbus >> 2) & 0x7)

struct pci_dev;

#ifdef CONFIG_PCI
extern void	iSeries_pcibios_init(void);
extern void	iSeries_pci_final_fixup(void);
extern void 	iSeries_pcibios_fixup_resources(struct pci_dev *dev);
#else
static inline void	iSeries_pcibios_init(void) { }
static inline void	iSeries_pci_final_fixup(void) { }
static inline void 	iSeries_pcibios_fixup_resources(struct pci_dev *dev) {}
#endif

#endif /* _PLATFORMS_ISERIES_PCI_H */
