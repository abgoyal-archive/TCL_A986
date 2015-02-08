

#undef DEBUG

#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/pci.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <asm/errno.h>
#include <asm/topology.h>
#include <asm/pci-bridge.h>
#include <asm/ppc-pci.h>
#include <asm/atomic.h>


static const struct of_device_id of_default_bus_ids[] = {
	{ .type = "soc", },
	{ .compatible = "soc", },
	{ .type = "spider", },
	{ .type = "axon", },
	{ .type = "plb5", },
	{ .type = "plb4", },
	{ .type = "opb", },
	{ .type = "ebc", },
	{},
};

struct bus_type of_platform_bus_type = {
       .uevent	= of_device_uevent,
};
EXPORT_SYMBOL(of_platform_bus_type);

static int __init of_bus_driver_init(void)
{
	return of_bus_type_init(&of_platform_bus_type, "of_platform");
}

postcore_initcall(of_bus_driver_init);

struct of_device* of_platform_device_create(struct device_node *np,
					    const char *bus_id,
					    struct device *parent)
{
	struct of_device *dev;

	dev = of_device_alloc(np, bus_id, parent);
	if (!dev)
		return NULL;

	dev->archdata.dma_mask = 0xffffffffUL;
	dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	dev->dev.bus = &of_platform_bus_type;

	/* We do not fill the DMA ops for platform devices by default.
	 * This is currently the responsibility of the platform code
	 * to do such, possibly using a device notifier
	 */

	if (of_device_register(dev) != 0) {
		of_device_free(dev);
		return NULL;
	}

	return dev;
}
EXPORT_SYMBOL(of_platform_device_create);



static int of_platform_bus_create(const struct device_node *bus,
				  const struct of_device_id *matches,
				  struct device *parent)
{
	struct device_node *child;
	struct of_device *dev;
	int rc = 0;

	for_each_child_of_node(bus, child) {
		pr_debug("   create child: %s\n", child->full_name);
		dev = of_platform_device_create(child, NULL, parent);
		if (dev == NULL)
			rc = -ENOMEM;
		else if (!of_match_node(matches, child))
			continue;
		if (rc == 0) {
			pr_debug("   and sub busses\n");
			rc = of_platform_bus_create(child, matches, &dev->dev);
		} if (rc) {
			of_node_put(child);
			break;
		}
	}
	return rc;
}


int of_platform_bus_probe(struct device_node *root,
			  const struct of_device_id *matches,
			  struct device *parent)
{
	struct device_node *child;
	struct of_device *dev;
	int rc = 0;

	if (matches == NULL)
		matches = of_default_bus_ids;
	if (matches == OF_NO_DEEP_PROBE)
		return -EINVAL;
	if (root == NULL)
		root = of_find_node_by_path("/");
	else
		of_node_get(root);

	pr_debug("of_platform_bus_probe()\n");
	pr_debug(" starting at: %s\n", root->full_name);

	/* Do a self check of bus type, if there's a match, create
	 * children
	 */
	if (of_match_node(matches, root)) {
		pr_debug(" root match, create all sub devices\n");
		dev = of_platform_device_create(root, NULL, parent);
		if (dev == NULL) {
			rc = -ENOMEM;
			goto bail;
		}
		pr_debug(" create all sub busses\n");
		rc = of_platform_bus_create(root, matches, &dev->dev);
		goto bail;
	}
	for_each_child_of_node(root, child) {
		if (!of_match_node(matches, child))
			continue;

		pr_debug("  match: %s\n", child->full_name);
		dev = of_platform_device_create(child, NULL, parent);
		if (dev == NULL)
			rc = -ENOMEM;
		else
			rc = of_platform_bus_create(child, matches, &dev->dev);
		if (rc) {
			of_node_put(child);
			break;
		}
	}
 bail:
	of_node_put(root);
	return rc;
}
EXPORT_SYMBOL(of_platform_bus_probe);

static int of_dev_node_match(struct device *dev, void *data)
{
	return to_of_device(dev)->dev.of_node == data;
}

struct of_device *of_find_device_by_node(struct device_node *np)
{
	struct device *dev;

	dev = bus_find_device(&of_platform_bus_type,
			      NULL, np, of_dev_node_match);
	if (dev)
		return to_of_device(dev);
	return NULL;
}
EXPORT_SYMBOL(of_find_device_by_node);

static int of_dev_phandle_match(struct device *dev, void *data)
{
	phandle *ph = data;
	return to_of_device(dev)->dev.of_node->phandle == *ph;
}

struct of_device *of_find_device_by_phandle(phandle ph)
{
	struct device *dev;

	dev = bus_find_device(&of_platform_bus_type,
			      NULL, &ph, of_dev_phandle_match);
	if (dev)
		return to_of_device(dev);
	return NULL;
}
EXPORT_SYMBOL(of_find_device_by_phandle);


#ifdef CONFIG_PPC_OF_PLATFORM_PCI


static int __devinit of_pci_phb_probe(struct of_device *dev,
				      const struct of_device_id *match)
{
	struct pci_controller *phb;

	/* Check if we can do that ... */
	if (ppc_md.pci_setup_phb == NULL)
		return -ENODEV;

	pr_info("Setting up PCI bus %s\n", dev->dev.of_node->full_name);

	/* Alloc and setup PHB data structure */
	phb = pcibios_alloc_controller(dev->dev.of_node);
	if (!phb)
		return -ENODEV;

	/* Setup parent in sysfs */
	phb->parent = &dev->dev;

	/* Setup the PHB using arch provided callback */
	if (ppc_md.pci_setup_phb(phb)) {
		pcibios_free_controller(phb);
		return -ENODEV;
	}

	/* Process "ranges" property */
	pci_process_bridge_OF_ranges(phb, dev->dev.of_node, 0);

	/* Init pci_dn data structures */
	pci_devs_phb_init_dynamic(phb);

	/* Register devices with EEH */
#ifdef CONFIG_EEH
	if (dev->dev.of_node->child)
		eeh_add_device_tree_early(dev->dev.of_node);
#endif /* CONFIG_EEH */

	/* Scan the bus */
	pcibios_scan_phb(phb, dev->dev.of_node);
	if (phb->bus == NULL)
		return -ENXIO;

	/* Claim resources. This might need some rework as well depending
	 * wether we are doing probe-only or not, like assigning unassigned
	 * resources etc...
	 */
	pcibios_claim_one_bus(phb->bus);

	/* Finish EEH setup */
#ifdef CONFIG_EEH
	eeh_add_device_tree_late(phb->bus);
#endif

	/* Add probed PCI devices to the device model */
	pci_bus_add_devices(phb->bus);

	return 0;
}

static struct of_device_id of_pci_phb_ids[] = {
	{ .type = "pci", },
	{ .type = "pcix", },
	{ .type = "pcie", },
	{ .type = "pciex", },
	{ .type = "ht", },
	{}
};

static struct of_platform_driver of_pci_phb_driver = {
	.probe = of_pci_phb_probe,
	.driver = {
		.name = "of-pci",
		.owner = THIS_MODULE,
		.of_match_table = of_pci_phb_ids,
	},
};

static __init int of_pci_phb_init(void)
{
	return of_register_platform_driver(&of_pci_phb_driver);
}

device_initcall(of_pci_phb_init);

#endif /* CONFIG_PPC_OF_PLATFORM_PCI */
