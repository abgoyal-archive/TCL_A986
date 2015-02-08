

#include <linux/efi.h>
#include <asm/xen/hypervisor.h>
#include <asm/xen/privop.h>

#include "irq_xen.h"

struct shared_info *HYPERVISOR_shared_info __read_mostly =
	(struct shared_info *)XSI_BASE;
EXPORT_SYMBOL(HYPERVISOR_shared_info);

DEFINE_PER_CPU(struct vcpu_info *, xen_vcpu);

struct start_info *xen_start_info;
EXPORT_SYMBOL(xen_start_info);

EXPORT_SYMBOL(xen_domain_type);

EXPORT_SYMBOL(__hypercall);

/* Stolen from arch/x86/xen/enlighten.c */

static void __init xen_vcpu_setup(int cpu)
{
	/*
	 * WARNING:
	 * before changing MAX_VIRT_CPUS,
	 * check that shared_info fits on a page
	 */
	BUILD_BUG_ON(sizeof(struct shared_info) > PAGE_SIZE);
	per_cpu(xen_vcpu, cpu) = &HYPERVISOR_shared_info->vcpu_info[cpu];
}

void __init xen_setup_vcpu_info_placement(void)
{
	int cpu;

	for_each_possible_cpu(cpu)
		xen_vcpu_setup(cpu);
}

void __cpuinit
xen_cpu_init(void)
{
	xen_smp_intr_init();
}

void
xen_ia64_enable_opt_feature(void)
{
	/* Enable region 7 identity map optimizations in Xen */
	struct xen_ia64_opt_feature optf;

	optf.cmd = XEN_IA64_OPTF_IDENT_MAP_REG7;
	optf.on = XEN_IA64_OPTF_ON;
	optf.pgprot = pgprot_val(PAGE_KERNEL);
	optf.key = 0;	/* No key on linux. */
	HYPERVISOR_opt_feature(&optf);
}
