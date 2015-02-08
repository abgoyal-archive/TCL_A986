
#ifndef _ASM_FIXMAP_H
#define _ASM_FIXMAP_H


extern unsigned long __FIXADDR_TOP;
#define FIXADDR_USER_START     __fix_to_virt(FIX_VDSO)
#define FIXADDR_USER_END       __fix_to_virt(FIX_VDSO - 1)

#ifndef __ASSEMBLY__
#include <linux/kernel.h>
#include <asm/acpi.h>
#include <asm/apicdef.h>
#include <asm/page.h>
#ifdef CONFIG_HIGHMEM
#include <linux/threads.h>
#include <asm/kmap_types.h>
#endif

enum fixed_addresses {
	FIX_HOLE,
	FIX_VDSO,
	FIX_DBGP_BASE,
	FIX_EARLYCON_MEM_BASE,
#ifdef CONFIG_X86_LOCAL_APIC
	FIX_APIC_BASE,	/* local (CPU) APIC) -- required for SMP or not */
#endif
#ifdef CONFIG_X86_IO_APIC
	FIX_IO_APIC_BASE_0,
	FIX_IO_APIC_BASE_END = FIX_IO_APIC_BASE_0 + MAX_IO_APICS-1,
#endif
#ifdef CONFIG_X86_VISWS_APIC
	FIX_CO_CPU,	/* Cobalt timer */
	FIX_CO_APIC,	/* Cobalt APIC Redirection Table */ 
	FIX_LI_PCIA,	/* Lithium PCI Bridge A */
	FIX_LI_PCIB,	/* Lithium PCI Bridge B */
#endif
#ifdef CONFIG_X86_F00F_BUG
	FIX_F00F_IDT,	/* Virtual mapping for IDT */
#endif
#ifdef CONFIG_X86_CYCLONE_TIMER
	FIX_CYCLONE_TIMER, /*cyclone timer register*/
#endif 
#ifdef CONFIG_HIGHMEM
	FIX_KMAP_BEGIN,	/* reserved pte's for temporary kernel mappings */
	FIX_KMAP_END = FIX_KMAP_BEGIN+(KM_TYPE_NR*NR_CPUS)-1,
#endif
#ifdef CONFIG_ACPI
	FIX_ACPI_BEGIN,
	FIX_ACPI_END = FIX_ACPI_BEGIN + FIX_ACPI_PAGES - 1,
#endif
#ifdef CONFIG_PCI_MMCONFIG
	FIX_PCIE_MCFG,
#endif
#ifdef CONFIG_PARAVIRT
	FIX_PARAVIRT_BOOTMAP,
#endif
	__end_of_permanent_fixed_addresses,
	/* temporary boot-time mappings, used before ioremap() is functional */
#define NR_FIX_BTMAPS	16
	FIX_BTMAP_END = __end_of_permanent_fixed_addresses,
	FIX_BTMAP_BEGIN = FIX_BTMAP_END + NR_FIX_BTMAPS - 1,
	FIX_WP_TEST,
	__end_of_fixed_addresses
};

extern void __set_fixmap (enum fixed_addresses idx,
					unsigned long phys, pgprot_t flags);
extern void reserve_top_address(unsigned long reserve);

#define set_fixmap(idx, phys) \
		__set_fixmap(idx, phys, PAGE_KERNEL)
#define set_fixmap_nocache(idx, phys) \
		__set_fixmap(idx, phys, PAGE_KERNEL_NOCACHE)

#define clear_fixmap(idx) \
		__set_fixmap(idx, 0, __pgprot(0))

#define FIXADDR_TOP	((unsigned long)__FIXADDR_TOP)

#define __FIXADDR_SIZE	(__end_of_permanent_fixed_addresses << PAGE_SHIFT)
#define __FIXADDR_BOOT_SIZE	(__end_of_fixed_addresses << PAGE_SHIFT)
#define FIXADDR_START		(FIXADDR_TOP - __FIXADDR_SIZE)
#define FIXADDR_BOOT_START	(FIXADDR_TOP - __FIXADDR_BOOT_SIZE)

#define __fix_to_virt(x)	(FIXADDR_TOP - ((x) << PAGE_SHIFT))
#define __virt_to_fix(x)	((FIXADDR_TOP - ((x)&PAGE_MASK)) >> PAGE_SHIFT)

extern void __this_fixmap_does_not_exist(void);

static __always_inline unsigned long fix_to_virt(const unsigned int idx)
{
	/*
	 * this branch gets completely eliminated after inlining,
	 * except when someone tries to use fixaddr indices in an
	 * illegal way. (such as mixing up address types or using
	 * out-of-range indices).
	 *
	 * If it doesn't get removed, the linker will complain
	 * loudly with a reasonably clear error message..
	 */
	if (idx >= __end_of_fixed_addresses)
		__this_fixmap_does_not_exist();

        return __fix_to_virt(idx);
}

static inline unsigned long virt_to_fix(const unsigned long vaddr)
{
	BUG_ON(vaddr >= FIXADDR_TOP || vaddr < FIXADDR_START);
	return __virt_to_fix(vaddr);
}

#endif /* !__ASSEMBLY__ */
#endif
