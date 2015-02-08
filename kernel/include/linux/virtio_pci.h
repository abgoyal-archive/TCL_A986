

#ifndef _LINUX_VIRTIO_PCI_H
#define _LINUX_VIRTIO_PCI_H

#include <linux/virtio_config.h>

/* A 32-bit r/o bitmask of the features supported by the host */
#define VIRTIO_PCI_HOST_FEATURES	0

/* A 32-bit r/w bitmask of features activated by the guest */
#define VIRTIO_PCI_GUEST_FEATURES	4

/* A 32-bit r/w PFN for the currently selected queue */
#define VIRTIO_PCI_QUEUE_PFN		8

/* A 16-bit r/o queue size for the currently selected queue */
#define VIRTIO_PCI_QUEUE_NUM		12

/* A 16-bit r/w queue selector */
#define VIRTIO_PCI_QUEUE_SEL		14

/* A 16-bit r/w queue notifier */
#define VIRTIO_PCI_QUEUE_NOTIFY		16

/* An 8-bit device status register.  */
#define VIRTIO_PCI_STATUS		18

#define VIRTIO_PCI_ISR			19

/* The bit of the ISR which indicates a device configuration change. */
#define VIRTIO_PCI_ISR_CONFIG		0x2

/* MSI-X registers: only enabled if MSI-X is enabled. */
/* A 16-bit vector for configuration changes. */
#define VIRTIO_MSI_CONFIG_VECTOR        20
/* A 16-bit vector for selected queue notifications. */
#define VIRTIO_MSI_QUEUE_VECTOR         22
/* Vector value used to disable MSI for queue */
#define VIRTIO_MSI_NO_VECTOR            0xffff

#define VIRTIO_PCI_CONFIG(dev)		((dev)->msix_enabled ? 24 : 20)

/* Virtio ABI version, this must match exactly */
#define VIRTIO_PCI_ABI_VERSION		0

#define VIRTIO_PCI_QUEUE_ADDR_SHIFT	12

#define VIRTIO_PCI_VRING_ALIGN		4096
#endif
