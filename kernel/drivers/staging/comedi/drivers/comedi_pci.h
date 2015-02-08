

#ifndef _COMEDI_PCI_H_
#define _COMEDI_PCI_H_

#include <linux/pci.h>

static inline int comedi_pci_enable(struct pci_dev *pdev, const char *res_name)
{
	int rc;

	rc = pci_enable_device(pdev);
	if (rc < 0)
		return rc;

	rc = pci_request_regions(pdev, res_name);
	if (rc < 0)
		pci_disable_device(pdev);

	return rc;
}

static inline void comedi_pci_disable(struct pci_dev *pdev)
{
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

#endif
