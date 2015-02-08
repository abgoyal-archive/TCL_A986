
#include <linux/pci.h>
#include <asm/ppc-pci.h>
#include <asm/pci-bridge.h>
#include <linux/kobject.h>

#define EEH_SHOW_ATTR(_name,_memb,_format)               \
static ssize_t eeh_show_##_name(struct device *dev,      \
		struct device_attribute *attr, char *buf)          \
{                                                        \
	struct pci_dev *pdev = to_pci_dev(dev);               \
	struct device_node *dn = pci_device_to_OF_node(pdev); \
	struct pci_dn *pdn;                                   \
	                                                      \
	if (!dn || PCI_DN(dn) == NULL)                        \
		return 0;                                          \
	                                                      \
	pdn = PCI_DN(dn);                                     \
	return sprintf(buf, _format "\n", pdn->_memb);        \
}                                                        \
static DEVICE_ATTR(_name, S_IRUGO, eeh_show_##_name, NULL);


EEH_SHOW_ATTR(eeh_mode, eeh_mode, "0x%x");
EEH_SHOW_ATTR(eeh_config_addr, eeh_config_addr, "0x%x");
EEH_SHOW_ATTR(eeh_pe_config_addr, eeh_pe_config_addr, "0x%x");
EEH_SHOW_ATTR(eeh_check_count, eeh_check_count, "%d");
EEH_SHOW_ATTR(eeh_freeze_count, eeh_freeze_count, "%d");
EEH_SHOW_ATTR(eeh_false_positives, eeh_false_positives, "%d");

void eeh_sysfs_add_device(struct pci_dev *pdev)
{
	int rc=0;

	rc += device_create_file(&pdev->dev, &dev_attr_eeh_mode);
	rc += device_create_file(&pdev->dev, &dev_attr_eeh_config_addr);
	rc += device_create_file(&pdev->dev, &dev_attr_eeh_pe_config_addr);
	rc += device_create_file(&pdev->dev, &dev_attr_eeh_check_count);
	rc += device_create_file(&pdev->dev, &dev_attr_eeh_false_positives);
	rc += device_create_file(&pdev->dev, &dev_attr_eeh_freeze_count);

	if (rc)
		printk(KERN_WARNING "EEH: Unable to create sysfs entries\n");
}

void eeh_sysfs_remove_device(struct pci_dev *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_eeh_mode);
	device_remove_file(&pdev->dev, &dev_attr_eeh_config_addr);
	device_remove_file(&pdev->dev, &dev_attr_eeh_pe_config_addr);
	device_remove_file(&pdev->dev, &dev_attr_eeh_check_count);
	device_remove_file(&pdev->dev, &dev_attr_eeh_false_positives);
	device_remove_file(&pdev->dev, &dev_attr_eeh_freeze_count);
}

