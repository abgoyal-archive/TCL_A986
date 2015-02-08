

#ifndef _CPCI_HOTPLUG_H
#define _CPCI_HOTPLUG_H

#include <linux/types.h>
#include <linux/pci.h>
#include <linux/pci_hotplug.h>

/* PICMG 2.1 R2.0 HS CSR bits: */
#define HS_CSR_INS	0x0080
#define HS_CSR_EXT	0x0040
#define HS_CSR_PI	0x0030
#define HS_CSR_LOO	0x0008
#define HS_CSR_PIE	0x0004
#define HS_CSR_EIM	0x0002
#define HS_CSR_DHA	0x0001

struct slot {
	u8 number;
	unsigned int devfn;
	struct pci_bus *bus;
	struct pci_dev *dev;
	unsigned int extracting;
	struct hotplug_slot *hotplug_slot;
	struct list_head slot_list;
};

struct cpci_hp_controller_ops {
	int (*query_enum) (void);
	int (*enable_irq) (void);
	int (*disable_irq) (void);
	int (*check_irq) (void *dev_id);
	int (*hardware_test) (struct slot* slot, u32 value);
	u8  (*get_power) (struct slot* slot);
	int (*set_power) (struct slot* slot, int value);
};

struct cpci_hp_controller {
	unsigned int irq;
	unsigned long irq_flags;
	char *devname;
	void *dev_id;
	char *name;
	struct cpci_hp_controller_ops *ops;
};

static inline const char *slot_name(struct slot *slot)
{
	return hotplug_slot_name(slot->hotplug_slot);
}

extern int cpci_hp_register_controller(struct cpci_hp_controller *controller);
extern int cpci_hp_unregister_controller(struct cpci_hp_controller *controller);
extern int cpci_hp_register_bus(struct pci_bus *bus, u8 first, u8 last);
extern int cpci_hp_unregister_bus(struct pci_bus *bus);
extern int cpci_hp_start(void);
extern int cpci_hp_stop(void);

extern u8 cpci_get_attention_status(struct slot *slot);
extern u8 cpci_get_latch_status(struct slot *slot);
extern u8 cpci_get_adapter_status(struct slot *slot);
extern u16 cpci_get_hs_csr(struct slot * slot);
extern int cpci_set_attention_status(struct slot *slot, int status);
extern int cpci_check_and_clear_ins(struct slot * slot);
extern int cpci_check_ext(struct slot * slot);
extern int cpci_clear_ext(struct slot * slot);
extern int cpci_led_on(struct slot * slot);
extern int cpci_led_off(struct slot * slot);
extern int cpci_configure_slot(struct slot *slot);
extern int cpci_unconfigure_slot(struct slot *slot);

#endif	/* _CPCI_HOTPLUG_H */
