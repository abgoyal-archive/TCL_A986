

#ifndef __SPARC_PBM_H
#define __SPARC_PBM_H

#include <linux/pci.h>
#include <asm/oplib.h>
#include <asm/prom.h>

struct linux_pbm_info {
	int		prom_node;
	char		prom_name[64];
	/* struct linux_prom_pci_ranges	pbm_ranges[PROMREG_MAX]; */
	/* int		num_pbm_ranges; */

	/* Now things for the actual PCI bus probes. */
	unsigned int	pci_first_busno;	/* Can it be nonzero? */
	struct pci_bus	*pci_bus;		/* Was inline, MJ allocs now */
};

struct pcidev_cookie {
	struct linux_pbm_info		*pbm;
	struct device_node		*prom_node;
};

#endif /* !(__SPARC_PBM_H) */
