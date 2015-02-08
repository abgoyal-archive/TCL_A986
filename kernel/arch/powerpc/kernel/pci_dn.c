
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/gfp.h>

#include <asm/io.h>
#include <asm/prom.h>
#include <asm/pci-bridge.h>
#include <asm/ppc-pci.h>
#include <asm/firmware.h>

void * __devinit update_dn_pci_info(struct device_node *dn, void *data)
{
	struct pci_controller *phb = data;
	const int *type =
		of_get_property(dn, "ibm,pci-config-space-type", NULL);
	const u32 *regs;
	struct pci_dn *pdn;

	pdn = alloc_maybe_bootmem(sizeof(*pdn), GFP_KERNEL);
	if (pdn == NULL)
		return NULL;
	memset(pdn, 0, sizeof(*pdn));
	dn->data = pdn;
	pdn->node = dn;
	pdn->phb = phb;
	regs = of_get_property(dn, "reg", NULL);
	if (regs) {
		/* First register entry is addr (00BBSS00)  */
		pdn->busno = (regs[0] >> 16) & 0xff;
		pdn->devfn = (regs[0] >> 8) & 0xff;
	}

	pdn->pci_ext_config_space = (type && *type == 1);
	return NULL;
}

void *traverse_pci_devices(struct device_node *start, traverse_func pre,
		void *data)
{
	struct device_node *dn, *nextdn;
	void *ret;

	/* We started with a phb, iterate all childs */
	for (dn = start->child; dn; dn = nextdn) {
		const u32 *classp;
		u32 class;

		nextdn = NULL;
		classp = of_get_property(dn, "class-code", NULL);
		class = classp ? *classp : 0;

		if (pre && ((ret = pre(dn, data)) != NULL))
			return ret;

		/* If we are a PCI bridge, go down */
		if (dn->child && ((class >> 8) == PCI_CLASS_BRIDGE_PCI ||
				  (class >> 8) == PCI_CLASS_BRIDGE_CARDBUS))
			/* Depth first...do children */
			nextdn = dn->child;
		else if (dn->sibling)
			/* ok, try next sibling instead. */
			nextdn = dn->sibling;
		if (!nextdn) {
			/* Walk up to next valid sibling. */
			do {
				dn = dn->parent;
				if (dn == start)
					return NULL;
			} while (dn->sibling == NULL);
			nextdn = dn->sibling;
		}
	}
	return NULL;
}

void __devinit pci_devs_phb_init_dynamic(struct pci_controller *phb)
{
	struct device_node *dn = phb->dn;
	struct pci_dn *pdn;

	/* PHB nodes themselves must not match */
	update_dn_pci_info(dn, phb);
	pdn = dn->data;
	if (pdn) {
		pdn->devfn = pdn->busno = -1;
		pdn->phb = phb;
	}

	/* Update dn->phb ptrs for new phb and children devices */
	traverse_pci_devices(dn, update_dn_pci_info, phb);
}

static void *is_devfn_node(struct device_node *dn, void *data)
{
	int busno = ((unsigned long)data >> 8) & 0xff;
	int devfn = ((unsigned long)data) & 0xff;
	struct pci_dn *pci = dn->data;

	if (pci && (devfn == pci->devfn) && (busno == pci->busno))
		return dn;
	return NULL;
}

struct device_node *fetch_dev_dn(struct pci_dev *dev)
{
	struct device_node *orig_dn = dev->sysdata;
	struct device_node *dn;
	unsigned long searchval = (dev->bus->number << 8) | dev->devfn;

	dn = traverse_pci_devices(orig_dn, is_devfn_node, (void *)searchval);
	if (dn)
		dev->sysdata = dn;
	return dn;
}
EXPORT_SYMBOL(fetch_dev_dn);

void __init pci_devs_phb_init(void)
{
	struct pci_controller *phb, *tmp;

	/* This must be done first so the device nodes have valid pci info! */
	list_for_each_entry_safe(phb, tmp, &hose_list, list_node)
		pci_devs_phb_init_dynamic(phb);
}
