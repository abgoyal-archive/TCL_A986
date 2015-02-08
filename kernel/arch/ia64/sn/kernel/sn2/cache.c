
#include <linux/module.h>
#include <asm/pgalloc.h>
#include <asm/sn/arch.h>

void
sn_flush_all_caches(long flush_addr, long bytes)
{
	unsigned long addr = flush_addr;

	/* SHub1 requires a cached address */
	if (is_shub1() && (addr & RGN_BITS) == RGN_BASE(RGN_UNCACHED))
		addr = (addr - RGN_BASE(RGN_UNCACHED)) + RGN_BASE(RGN_KERNEL);

	flush_icache_range(addr, addr + bytes);
	/*
	 * The last call may have returned before the caches
	 * were actually flushed, so we call it again to make
	 * sure.
	 */
	flush_icache_range(addr, addr + bytes);
	mb();
}
EXPORT_SYMBOL(sn_flush_all_caches);
