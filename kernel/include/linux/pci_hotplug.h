
#ifndef _PCI_HOTPLUG_H
#define _PCI_HOTPLUG_H

/* These values come from the PCI Express Spec */
enum pcie_link_width {
	PCIE_LNK_WIDTH_RESRV	= 0x00,
	PCIE_LNK_X1		= 0x01,
	PCIE_LNK_X2		= 0x02,
	PCIE_LNK_X4		= 0x04,
	PCIE_LNK_X8		= 0x08,
	PCIE_LNK_X12		= 0x0C,
	PCIE_LNK_X16		= 0x10,
	PCIE_LNK_X32		= 0x20,
	PCIE_LNK_WIDTH_UNKNOWN  = 0xFF,
};

struct hotplug_slot_ops {
	struct module *owner;
	const char *mod_name;
	int (*enable_slot)		(struct hotplug_slot *slot);
	int (*disable_slot)		(struct hotplug_slot *slot);
	int (*set_attention_status)	(struct hotplug_slot *slot, u8 value);
	int (*hardware_test)		(struct hotplug_slot *slot, u32 value);
	int (*get_power_status)		(struct hotplug_slot *slot, u8 *value);
	int (*get_attention_status)	(struct hotplug_slot *slot, u8 *value);
	int (*get_latch_status)		(struct hotplug_slot *slot, u8 *value);
	int (*get_adapter_status)	(struct hotplug_slot *slot, u8 *value);
};

struct hotplug_slot_info {
	u8	power_status;
	u8	attention_status;
	u8	latch_status;
	u8	adapter_status;
};

struct hotplug_slot {
	struct hotplug_slot_ops		*ops;
	struct hotplug_slot_info	*info;
	void (*release) (struct hotplug_slot *slot);
	void				*private;

	/* Variables below this are for use only by the hotplug pci core. */
	struct list_head		slot_list;
	struct pci_slot			*pci_slot;
};
#define to_hotplug_slot(n) container_of(n, struct hotplug_slot, kobj)

static inline const char *hotplug_slot_name(const struct hotplug_slot *slot)
{
	return pci_slot_name(slot->pci_slot);
}

extern int __pci_hp_register(struct hotplug_slot *slot, struct pci_bus *pbus,
			     int nr, const char *name,
			     struct module *owner, const char *mod_name);
extern int pci_hp_deregister(struct hotplug_slot *slot);
extern int __must_check pci_hp_change_slot_info	(struct hotplug_slot *slot,
						 struct hotplug_slot_info *info);

static inline int pci_hp_register(struct hotplug_slot *slot,
				  struct pci_bus *pbus,
				  int devnr, const char *name)
{
	return __pci_hp_register(slot, pbus, devnr, name,
				 THIS_MODULE, KBUILD_MODNAME);
}

/* PCI Setting Record (Type 0) */
struct hpp_type0 {
	u32 revision;
	u8  cache_line_size;
	u8  latency_timer;
	u8  enable_serr;
	u8  enable_perr;
};

/* PCI-X Setting Record (Type 1) */
struct hpp_type1 {
	u32 revision;
	u8  max_mem_read;
	u8  avg_max_split;
	u16 tot_max_split;
};

/* PCI Express Setting Record (Type 2) */
struct hpp_type2 {
	u32 revision;
	u32 unc_err_mask_and;
	u32 unc_err_mask_or;
	u32 unc_err_sever_and;
	u32 unc_err_sever_or;
	u32 cor_err_mask_and;
	u32 cor_err_mask_or;
	u32 adv_err_cap_and;
	u32 adv_err_cap_or;
	u16 pci_exp_devctl_and;
	u16 pci_exp_devctl_or;
	u16 pci_exp_lnkctl_and;
	u16 pci_exp_lnkctl_or;
	u32 sec_unc_err_sever_and;
	u32 sec_unc_err_sever_or;
	u32 sec_unc_err_mask_and;
	u32 sec_unc_err_mask_or;
};

struct hotplug_params {
	struct hpp_type0 *t0;		/* Type0: NULL if not available */
	struct hpp_type1 *t1;		/* Type1: NULL if not available */
	struct hpp_type2 *t2;		/* Type2: NULL if not available */
	struct hpp_type0 type0_data;
	struct hpp_type1 type1_data;
	struct hpp_type2 type2_data;
};

#ifdef CONFIG_ACPI
#include <acpi/acpi.h>
#include <acpi/acpi_bus.h>
int pci_get_hp_params(struct pci_dev *dev, struct hotplug_params *hpp);
int acpi_get_hp_hw_control_from_firmware(struct pci_dev *dev, u32 flags);
int acpi_pci_check_ejectable(struct pci_bus *pbus, acpi_handle handle);
int acpi_pci_detect_ejectable(acpi_handle handle);
#else
static inline int pci_get_hp_params(struct pci_dev *dev,
				    struct hotplug_params *hpp)
{
	return -ENODEV;
}
#endif

void pci_configure_slot(struct pci_dev *dev);
#endif

