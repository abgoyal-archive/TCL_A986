
#ifndef _HPIOS_H_
#define _HPIOS_H_

#undef HPI_OS_LINUX_KERNEL
#define HPI_OS_LINUX_KERNEL

#define HPI_OS_DEFINED
#define HPI_KERNEL_MODE

#define HPI_REASSIGN_DUPLICATE_ADAPTER_IDX

#include <linux/io.h>
#include <asm/system.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/pci.h>

#define HPI_NO_OS_FILE_OPS

#ifdef CONFIG_64BIT
#define HPI64BIT
#endif

struct consistent_dma_area {
	struct device *pdev;
	/* looks like dma-mapping dma_devres ?! */
	size_t size;
	void *vaddr;
	dma_addr_t dma_handle;
};

static inline u16 hpios_locked_mem_get_phys_addr(struct consistent_dma_area
	*locked_mem_handle, u32 *p_physical_addr)
{
	*p_physical_addr = locked_mem_handle->dma_handle;
	return 0;
}

static inline u16 hpios_locked_mem_get_virt_addr(struct consistent_dma_area
	*locked_mem_handle, void **pp_virtual_addr)
{
	*pp_virtual_addr = locked_mem_handle->vaddr;
	return 0;
}

static inline u16 hpios_locked_mem_valid(struct consistent_dma_area
	*locked_mem_handle)
{
	return locked_mem_handle->size != 0;
}

struct hpi_ioctl_linux {
	void __user *phm;
	void __user *phr;
};

#define HPI_IOCTL_LINUX _IOWR('H', 0xFC, struct hpi_ioctl_linux)

#define HPI_DEBUG_FLAG_ERROR   KERN_ERR
#define HPI_DEBUG_FLAG_WARNING KERN_WARNING
#define HPI_DEBUG_FLAG_NOTICE  KERN_NOTICE
#define HPI_DEBUG_FLAG_INFO    KERN_INFO
#define HPI_DEBUG_FLAG_DEBUG   KERN_DEBUG
#define HPI_DEBUG_FLAG_VERBOSE KERN_DEBUG	/* kernel has no verbose */

#include <linux/spinlock.h>

#define HPI_LOCKING

struct hpios_spinlock {
	spinlock_t lock;	/* SEE hpios_spinlock */
	int lock_context;
};

#define IN_LOCK_BH 1
#define IN_LOCK_IRQ 0
static inline void cond_lock(struct hpios_spinlock *l)
{
	if (irqs_disabled()) {
		/* NO bh or isr can execute on this processor,
		   so ordinary lock will do
		 */
		spin_lock(&((l)->lock));
		l->lock_context = IN_LOCK_IRQ;
	} else {
		spin_lock_bh(&((l)->lock));
		l->lock_context = IN_LOCK_BH;
	}
}

static inline void cond_unlock(struct hpios_spinlock *l)
{
	if (l->lock_context == IN_LOCK_BH)
		spin_unlock_bh(&((l)->lock));
	else
		spin_unlock(&((l)->lock));
}

#define hpios_msgxlock_init(obj)      spin_lock_init(&(obj)->lock)
#define hpios_msgxlock_lock(obj)   cond_lock(obj)
#define hpios_msgxlock_un_lock(obj) cond_unlock(obj)

#define hpios_dsplock_init(obj)       spin_lock_init(&(obj)->dsp_lock.lock)
#define hpios_dsplock_lock(obj)    cond_lock(&(obj)->dsp_lock)
#define hpios_dsplock_unlock(obj)  cond_unlock(&(obj)->dsp_lock)

#ifdef CONFIG_SND_DEBUG
#define HPI_DEBUG
#endif

#define HPI_ALIST_LOCKING
#define hpios_alistlock_init(obj)    spin_lock_init(&((obj)->list_lock.lock))
#define hpios_alistlock_lock(obj) spin_lock(&((obj)->list_lock.lock))
#define hpios_alistlock_un_lock(obj) spin_unlock(&((obj)->list_lock.lock))

struct hpi_adapter {
	/* mutex prevents contention for one card
	   between multiple user programs (via ioctl) */
	struct mutex mutex;
	u16 index;
	u16 type;

	/* ALSA card structure */
	void *snd_card_asihpi;

	char *p_buffer;
	size_t buffer_size;
	struct pci_dev *pci;
	void __iomem *ap_remapped_mem_base[HPI_MAX_ADAPTER_MEM_SPACES];
};

#endif
