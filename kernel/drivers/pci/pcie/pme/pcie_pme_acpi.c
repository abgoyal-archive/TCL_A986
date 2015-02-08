

#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/acpi.h>
#include <linux/pci-acpi.h>
#include <linux/pcieport_if.h>

int pcie_pme_acpi_setup(struct pcie_device *srv)
{
	acpi_status status = AE_NOT_FOUND;
	struct pci_dev *port = srv->port;
	acpi_handle handle;
	int error = 0;

	if (acpi_pci_disabled)
		return -ENOSYS;

	dev_info(&port->dev, "Requesting control of PCIe PME from ACPI BIOS\n");

	handle = acpi_find_root_bridge_handle(port);
	if (!handle)
		return -EINVAL;

	status = acpi_pci_osc_control_set(handle,
			OSC_PCI_EXPRESS_PME_CONTROL |
			OSC_PCI_EXPRESS_CAP_STRUCTURE_CONTROL);
	if (ACPI_FAILURE(status)) {
		dev_info(&port->dev,
			"Failed to receive control of PCIe PME service: %s\n",
			(status == AE_SUPPORT || status == AE_NOT_FOUND) ?
			"no _OSC support" : "ACPI _OSC failed");
		error = -ENODEV;
	}

	return error;
}
