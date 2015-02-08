

#ifndef __XEN_PUBLIC_FEATURES_H__
#define __XEN_PUBLIC_FEATURES_H__

#define XENFEAT_writable_page_tables       0

#define XENFEAT_writable_descriptor_tables 1

#define XENFEAT_auto_translated_physmap    2

/* If set, the guest is running in supervisor mode (e.g., x86 ring 0). */
#define XENFEAT_supervisor_mode_kernel     3

#define XENFEAT_pae_pgdir_above_4gb        4

/* x86: Does this Xen host support the MMU_PT_UPDATE_PRESERVE_AD hypercall? */
#define XENFEAT_mmu_pt_update_preserve_ad  5

#define XENFEAT_NR_SUBMAPS 1

#endif /* __XEN_PUBLIC_FEATURES_H__ */
