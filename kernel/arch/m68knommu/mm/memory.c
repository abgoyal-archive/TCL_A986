

#include <linux/mm.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>

#include <asm/segment.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/system.h>


unsigned long kernel_map(unsigned long paddr, unsigned long size,
			 int nocacheflag, unsigned long *memavailp )
{
	return paddr;
}

