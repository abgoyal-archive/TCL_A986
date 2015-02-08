
#include <linux/topology.h>
#include <linux/module.h>
#include <asm/processor.h>
#include <asm/smp.h>

u16 cpu_to_node_map[NR_CPUS] __cacheline_aligned;
EXPORT_SYMBOL(cpu_to_node_map);

cpumask_t node_to_cpu_mask[MAX_NUMNODES] __cacheline_aligned;
EXPORT_SYMBOL(node_to_cpu_mask);

void __cpuinit map_cpu_to_node(int cpu, int nid)
{
	int oldnid;
	if (nid < 0) { /* just initialize by zero */
		cpu_to_node_map[cpu] = 0;
		return;
	}
	/* sanity check first */
	oldnid = cpu_to_node_map[cpu];
	if (cpu_isset(cpu, node_to_cpu_mask[oldnid])) {
		return; /* nothing to do */
	}
	/* we don't have cpu-driven node hot add yet...
	   In usual case, node is created from SRAT at boot time. */
	if (!node_online(nid))
		nid = first_online_node;
	cpu_to_node_map[cpu] = nid;
	cpu_set(cpu, node_to_cpu_mask[nid]);
	return;
}

void __cpuinit unmap_cpu_from_node(int cpu, int nid)
{
	WARN_ON(!cpu_isset(cpu, node_to_cpu_mask[nid]));
	WARN_ON(cpu_to_node_map[cpu] != nid);
	cpu_to_node_map[cpu] = 0;
	cpu_clear(cpu, node_to_cpu_mask[nid]);
}


void __init build_cpu_to_node_map(void)
{
	int cpu, i, node;

	for(node=0; node < MAX_NUMNODES; node++)
		cpus_clear(node_to_cpu_mask[node]);

	for_each_possible_early_cpu(cpu) {
		node = -1;
		for (i = 0; i < NR_CPUS; ++i)
			if (cpu_physical_id(cpu) == node_cpuid[i].phys_id) {
				node = node_cpuid[i].nid;
				break;
			}
		map_cpu_to_node(cpu, node);
	}
}
