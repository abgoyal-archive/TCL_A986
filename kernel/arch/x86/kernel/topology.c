
#include <linux/nodemask.h>
#include <linux/mmzone.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <asm/cpu.h>

static DEFINE_PER_CPU(struct x86_cpu, cpu_devices);

#ifdef CONFIG_HOTPLUG_CPU
int __ref arch_register_cpu(int num)
{
	/*
	 * CPU0 cannot be offlined due to several
	 * restrictions and assumptions in kernel. This basically
	 * doesnt add a control file, one cannot attempt to offline
	 * BSP.
	 *
	 * Also certain PCI quirks require not to enable hotplug control
	 * for all CPU's.
	 */
	if (num)
		per_cpu(cpu_devices, num).cpu.hotpluggable = 1;

	return register_cpu(&per_cpu(cpu_devices, num).cpu, num);
}
EXPORT_SYMBOL(arch_register_cpu);

void arch_unregister_cpu(int num)
{
	unregister_cpu(&per_cpu(cpu_devices, num).cpu);
}
EXPORT_SYMBOL(arch_unregister_cpu);
#else /* CONFIG_HOTPLUG_CPU */

static int __init arch_register_cpu(int num)
{
	return register_cpu(&per_cpu(cpu_devices, num).cpu, num);
}
#endif /* CONFIG_HOTPLUG_CPU */

static int __init topology_init(void)
{
	int i;

#ifdef CONFIG_NUMA
	for_each_online_node(i)
		register_one_node(i);
#endif

	for_each_present_cpu(i)
		arch_register_cpu(i);

	return 0;
}
subsys_initcall(topology_init);
