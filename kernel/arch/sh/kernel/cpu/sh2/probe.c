
#include <linux/init.h>
#include <asm/processor.h>
#include <asm/cache.h>

void __cpuinit cpu_probe(void)
{
#if defined(CONFIG_CPU_SUBTYPE_SH7619)
	boot_cpu_data.type			= CPU_SH7619;
	boot_cpu_data.dcache.ways		= 4;
	boot_cpu_data.dcache.way_incr	= (1<<12);
	boot_cpu_data.dcache.sets		= 256;
	boot_cpu_data.dcache.entry_shift	= 4;
	boot_cpu_data.dcache.linesz		= L1_CACHE_BYTES;
	boot_cpu_data.dcache.flags		= 0;
#endif
	/*
	 * SH-2 doesn't have separate caches
	 */
	boot_cpu_data.dcache.flags |= SH_CACHE_COMBINED;
	boot_cpu_data.icache = boot_cpu_data.dcache;
	boot_cpu_data.family = CPU_FAMILY_SH2;
}
