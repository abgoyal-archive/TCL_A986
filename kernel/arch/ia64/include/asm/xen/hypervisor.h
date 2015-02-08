

#ifndef _ASM_IA64_XEN_HYPERVISOR_H
#define _ASM_IA64_XEN_HYPERVISOR_H

#include <linux/err.h>
#include <xen/interface/xen.h>
#include <xen/interface/version.h>	/* to compile feature.c */
#include <xen/features.h>		/* to comiple xen-netfront.c */
#include <xen/xen.h>
#include <asm/xen/hypercall.h>

#ifdef CONFIG_XEN
extern struct shared_info *HYPERVISOR_shared_info;
extern struct start_info *xen_start_info;

void __init xen_setup_vcpu_info_placement(void);
void force_evtchn_callback(void);

/* for drivers/xen/balloon/balloon.c */
#ifdef CONFIG_XEN_SCRUB_PAGES
#define scrub_pages(_p, _n) memset((void *)(_p), 0, (_n) << PAGE_SHIFT)
#else
#define scrub_pages(_p, _n) ((void)0)
#endif

/* For setup_arch() in arch/ia64/kernel/setup.c */
void xen_ia64_enable_opt_feature(void);
#endif

#endif /* _ASM_IA64_XEN_HYPERVISOR_H */
