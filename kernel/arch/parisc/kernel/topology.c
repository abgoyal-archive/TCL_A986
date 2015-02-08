

#include <linux/init.h>
#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/cache.h>

static DEFINE_PER_CPU(struct cpu, cpu_devices);

static int __init topology_init(void)
{
	int num;

	for_each_present_cpu(num) {
		register_cpu(&per_cpu(cpu_devices, num), num);
	}
	return 0;
}

subsys_initcall(topology_init);
