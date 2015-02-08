

#include <linux/mm.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>

#include <asm/setup.h>
#include <asm/segment.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/traps.h>
#include <asm/io.h>

void cache_clear (unsigned long paddr, int len)
{
}


void cache_push (unsigned long paddr, int len)
{
}

void cache_push_v (unsigned long vaddr, int len)
{
}


unsigned long kernel_map(unsigned long paddr, unsigned long size,
			 int nocacheflag, unsigned long *memavailp )
{
	return paddr;
}

