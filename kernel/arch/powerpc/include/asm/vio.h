

#ifndef _ASM_POWERPC_VIO_H
#define _ASM_POWERPC_VIO_H
#ifdef __KERNEL__

#include <linux/init.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/mod_devicetable.h>

#include <asm/hvcall.h>
#include <asm/scatterlist.h>

#define VETH_MAC_ADDR "local-mac-address"
#define VETH_MCAST_FILTER_SIZE "ibm,mac-address-filters"

/* End architecture-specific constants */

#define h_vio_signal(ua, mode) \
  plpar_hcall_norets(H_VIO_SIGNAL, ua, mode)

#define VIO_IRQ_DISABLE		0UL
#define VIO_IRQ_ENABLE		1UL

#define VIO_CMO_MIN_ENT 1562624

struct iommu_table;

struct vio_dev {
	const char *name;
	const char *type;
	uint32_t unit_address;
	unsigned int irq;
	struct {
		size_t desired;
		size_t entitled;
		size_t allocated;
		atomic_t allocs_failed;
	} cmo;
	struct device dev;
};

struct vio_driver {
	const struct vio_device_id *id_table;
	int (*probe)(struct vio_dev *dev, const struct vio_device_id *id);
	int (*remove)(struct vio_dev *dev);
	/* A driver must have a get_desired_dma() function to
	 * be loaded in a CMO environment if it uses DMA.
	 */
	unsigned long (*get_desired_dma)(struct vio_dev *dev);
	struct device_driver driver;
};

extern int vio_register_driver(struct vio_driver *drv);
extern void vio_unregister_driver(struct vio_driver *drv);

extern int vio_cmo_entitlement_update(size_t);
extern void vio_cmo_set_dev_desired(struct vio_dev *viodev, size_t desired);

extern void __devinit vio_unregister_device(struct vio_dev *dev);

struct device_node;

extern struct vio_dev *vio_register_device_node(
		struct device_node *node_vdev);
extern const void *vio_get_attribute(struct vio_dev *vdev, char *which,
		int *length);
#ifdef CONFIG_PPC_PSERIES
extern struct vio_dev *vio_find_node(struct device_node *vnode);
extern int vio_enable_interrupts(struct vio_dev *dev);
extern int vio_disable_interrupts(struct vio_dev *dev);
#else
static inline int vio_enable_interrupts(struct vio_dev *dev)
{
	return 0;
}
#endif

static inline struct vio_driver *to_vio_driver(struct device_driver *drv)
{
	return container_of(drv, struct vio_driver, driver);
}

static inline struct vio_dev *to_vio_dev(struct device *dev)
{
	return container_of(dev, struct vio_dev, dev);
}

#endif /* __KERNEL__ */
#endif /* _ASM_POWERPC_VIO_H */
