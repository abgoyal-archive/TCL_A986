


#ifndef RT2X00PCI_H
#define RT2X00PCI_H

#include <linux/io.h>
#include <linux/pci.h>

#define PCI_DEVICE_DATA(__ops)	.driver_data = (kernel_ulong_t)(__ops)

static inline void rt2x00pci_register_read(struct rt2x00_dev *rt2x00dev,
					   const unsigned int offset,
					   u32 *value)
{
	*value = readl(rt2x00dev->csr.base + offset);
}

static inline void rt2x00pci_register_multiread(struct rt2x00_dev *rt2x00dev,
						const unsigned int offset,
						void *value, const u32 length)
{
	memcpy_fromio(value, rt2x00dev->csr.base + offset, length);
}

static inline void rt2x00pci_register_write(struct rt2x00_dev *rt2x00dev,
					    const unsigned int offset,
					    u32 value)
{
	writel(value, rt2x00dev->csr.base + offset);
}

static inline void rt2x00pci_register_multiwrite(struct rt2x00_dev *rt2x00dev,
						 const unsigned int offset,
						 const void *value,
						 const u32 length)
{
	memcpy_toio(rt2x00dev->csr.base + offset, value, length);
}

int rt2x00pci_regbusy_read(struct rt2x00_dev *rt2x00dev,
			   const unsigned int offset,
			   const struct rt2x00_field32 field,
			   u32 *reg);

int rt2x00pci_write_tx_data(struct queue_entry *entry,
			    struct txentry_desc *txdesc);

struct queue_entry_priv_pci {
	__le32 *desc;
	dma_addr_t desc_dma;
};

void rt2x00pci_rxdone(struct rt2x00_dev *rt2x00dev);

int rt2x00pci_initialize(struct rt2x00_dev *rt2x00dev);
void rt2x00pci_uninitialize(struct rt2x00_dev *rt2x00dev);

int rt2x00pci_probe(struct pci_dev *pci_dev, const struct pci_device_id *id);
void rt2x00pci_remove(struct pci_dev *pci_dev);
#ifdef CONFIG_PM
int rt2x00pci_suspend(struct pci_dev *pci_dev, pm_message_t state);
int rt2x00pci_resume(struct pci_dev *pci_dev);
#else
#define rt2x00pci_suspend	NULL
#define rt2x00pci_resume	NULL
#endif /* CONFIG_PM */

#endif /* RT2X00PCI_H */
