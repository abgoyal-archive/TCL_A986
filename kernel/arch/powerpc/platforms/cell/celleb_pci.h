

#ifndef _CELLEB_PCI_H
#define _CELLEB_PCI_H

#include <linux/pci.h>

#include <asm/pci-bridge.h>
#include <asm/prom.h>
#include <asm/ppc-pci.h>

#include "io-workarounds.h"

struct celleb_phb_spec {
	int (*setup)(struct device_node *, struct pci_controller *);
	struct ppc_pci_io *ops;
	int (*iowa_init)(struct iowa_bus *, void *);
	void *iowa_data;
};

extern int celleb_setup_phb(struct pci_controller *);
extern int celleb_pci_probe_mode(struct pci_bus *);

extern struct celleb_phb_spec celleb_epci_spec;
extern struct celleb_phb_spec celleb_pciex_spec;

#endif /* _CELLEB_PCI_H */
