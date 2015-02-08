

#include <linux/module.h>
#include <asm/processor.h>
#include <asm/hypervisor.h>

static const __initconst struct hypervisor_x86 * const hypervisors[] =
{
	&x86_hyper_vmware,
	&x86_hyper_ms_hyperv,
};

const struct hypervisor_x86 *x86_hyper;
EXPORT_SYMBOL(x86_hyper);

static inline void __init
detect_hypervisor_vendor(void)
{
	const struct hypervisor_x86 *h, * const *p;

	for (p = hypervisors; p < hypervisors + ARRAY_SIZE(hypervisors); p++) {
		h = *p;
		if (h->detect()) {
			x86_hyper = h;
			printk(KERN_INFO "Hypervisor detected: %s\n", h->name);
			break;
		}
	}
}

void __cpuinit init_hypervisor(struct cpuinfo_x86 *c)
{
	if (x86_hyper && x86_hyper->set_cpu_features)
		x86_hyper->set_cpu_features(c);
}

void __init init_hypervisor_platform(void)
{

	detect_hypervisor_vendor();

	if (!x86_hyper)
		return;

	init_hypervisor(&boot_cpu_data);

	if (x86_hyper->init_platform)
		x86_hyper->init_platform();
}
