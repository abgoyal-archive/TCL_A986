

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/stddef.h>
#include <linux/init.h>
#include <linux/bootmem.h>
#include <linux/highmem.h>
#include <linux/initrd.h>
#include <linux/pagemap.h>
#include <linux/memblock.h>
#include <linux/gfp.h>

#include <asm/pgalloc.h>
#include <asm/prom.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/mmu.h>
#include <asm/smp.h>
#include <asm/machdep.h>
#include <asm/btext.h>
#include <asm/tlb.h>
#include <asm/sections.h>
#include <asm/system.h>

#include "mmu_decl.h"

#if defined(CONFIG_KERNEL_START_BOOL) || defined(CONFIG_LOWMEM_SIZE_BOOL)
/* The amount of lowmem must be within 0xF0000000 - KERNELBASE. */
#if (CONFIG_LOWMEM_SIZE > (0xF0000000 - PAGE_OFFSET))
#error "You must adjust CONFIG_LOWMEM_SIZE or CONFIG_START_KERNEL"
#endif
#endif
#define MAX_LOW_MEM	CONFIG_LOWMEM_SIZE

phys_addr_t total_memory;
phys_addr_t total_lowmem;

phys_addr_t memstart_addr = (phys_addr_t)~0ull;
EXPORT_SYMBOL(memstart_addr);
phys_addr_t kernstart_addr;
EXPORT_SYMBOL(kernstart_addr);
phys_addr_t lowmem_end_addr;

int boot_mapsize;
#ifdef CONFIG_PPC_PMAC
unsigned long agp_special_page;
EXPORT_SYMBOL(agp_special_page);
#endif

void MMU_init(void);

/* XXX should be in current.h  -- paulus */
extern struct task_struct *current_set[NR_CPUS];

int __map_without_bats;
int __map_without_ltlbs;

int __allow_ioremap_reserved;

/* max amount of low RAM to map in */
unsigned long __max_low_memory = MAX_LOW_MEM;

phys_addr_t __initial_memory_limit_addr = (phys_addr_t)0x10000000;

void MMU_setup(void)
{
	/* Check for nobats option (used in mapin_ram). */
	if (strstr(cmd_line, "nobats")) {
		__map_without_bats = 1;
	}

	if (strstr(cmd_line, "noltlbs")) {
		__map_without_ltlbs = 1;
	}
#ifdef CONFIG_DEBUG_PAGEALLOC
	__map_without_bats = 1;
	__map_without_ltlbs = 1;
#endif
}

void __init MMU_init(void)
{
	if (ppc_md.progress)
		ppc_md.progress("MMU:enter", 0x111);

	/* 601 can only access 16MB at the moment */
	if (PVR_VER(mfspr(SPRN_PVR)) == 1)
		__initial_memory_limit_addr = 0x01000000;
	/* 8xx can only access 8MB at the moment */
	if (PVR_VER(mfspr(SPRN_PVR)) == 0x50)
		__initial_memory_limit_addr = 0x00800000;

	/* parse args from command line */
	MMU_setup();

	if (memblock.memory.cnt > 1) {
#ifndef CONFIG_WII
		memblock.memory.cnt = 1;
		memblock_analyze();
		printk(KERN_WARNING "Only using first contiguous memory region");
#else
		wii_memory_fixups();
#endif
	}

	total_lowmem = total_memory = memblock_end_of_DRAM() - memstart_addr;
	lowmem_end_addr = memstart_addr + total_lowmem;

#ifdef CONFIG_FSL_BOOKE
	/* Freescale Book-E parts expect lowmem to be mapped by fixed TLB
	 * entries, so we need to adjust lowmem to match the amount we can map
	 * in the fixed entries */
	adjust_total_lowmem();
#endif /* CONFIG_FSL_BOOKE */

	if (total_lowmem > __max_low_memory) {
		total_lowmem = __max_low_memory;
		lowmem_end_addr = memstart_addr + total_lowmem;
#ifndef CONFIG_HIGHMEM
		total_memory = total_lowmem;
		memblock_enforce_memory_limit(lowmem_end_addr);
		memblock_analyze();
#endif /* CONFIG_HIGHMEM */
	}

	/* Initialize the MMU hardware */
	if (ppc_md.progress)
		ppc_md.progress("MMU:hw init", 0x300);
	MMU_init_hw();

	/* Map in all of RAM starting at KERNELBASE */
	if (ppc_md.progress)
		ppc_md.progress("MMU:mapin", 0x301);
	mapin_ram();

	/* Initialize early top-down ioremap allocator */
	ioremap_bot = IOREMAP_TOP;

	/* Map in I/O resources */
	if (ppc_md.progress)
		ppc_md.progress("MMU:setio", 0x302);

	if (ppc_md.progress)
		ppc_md.progress("MMU:exit", 0x211);

	/* From now on, btext is no longer BAT mapped if it was at all */
#ifdef CONFIG_BOOTX_TEXT
	btext_unmap();
#endif
}

/* This is only called until mem_init is done. */
void __init *early_get_page(void)
{
	void *p;

	if (init_bootmem_done) {
		p = alloc_bootmem_pages(PAGE_SIZE);
	} else {
		p = __va(memblock_alloc_base(PAGE_SIZE, PAGE_SIZE,
					__initial_memory_limit_addr));
	}
	return p;
}

/* Free up now-unused memory */
static void free_sec(unsigned long start, unsigned long end, const char *name)
{
	unsigned long cnt = 0;

	while (start < end) {
		ClearPageReserved(virt_to_page(start));
		init_page_count(virt_to_page(start));
		free_page(start);
		cnt++;
		start += PAGE_SIZE;
 	}
	if (cnt) {
		printk(" %ldk %s", cnt << (PAGE_SHIFT - 10), name);
		totalram_pages += cnt;
	}
}

void free_initmem(void)
{
#define FREESEC(TYPE) \
	free_sec((unsigned long)(&__ ## TYPE ## _begin), \
		 (unsigned long)(&__ ## TYPE ## _end), \
		 #TYPE);

	printk ("Freeing unused kernel memory:");
	FREESEC(init);
 	printk("\n");
	ppc_md.progress = NULL;
#undef FREESEC
}

#ifdef CONFIG_BLK_DEV_INITRD
void free_initrd_mem(unsigned long start, unsigned long end)
{
	if (start < end)
		printk ("Freeing initrd memory: %ldk freed\n", (end - start) >> 10);
	for (; start < end; start += PAGE_SIZE) {
		ClearPageReserved(virt_to_page(start));
		init_page_count(virt_to_page(start));
		free_page(start);
		totalram_pages++;
	}
}
#endif

