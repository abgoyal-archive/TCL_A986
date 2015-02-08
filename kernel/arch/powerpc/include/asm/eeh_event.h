

#ifndef ASM_POWERPC_EEH_EVENT_H
#define ASM_POWERPC_EEH_EVENT_H
#ifdef __KERNEL__

struct eeh_event {
	struct list_head     list;
	struct device_node 	*dn;   /* struct device node */
	struct pci_dev       *dev;  /* affected device */
};

int eeh_send_failure_event (struct device_node *dn,
                            struct pci_dev *dev);

/* Main recovery function */
struct pci_dn * handle_eeh_events (struct eeh_event *);

#endif /* __KERNEL__ */
#endif /* ASM_POWERPC_EEH_EVENT_H */
