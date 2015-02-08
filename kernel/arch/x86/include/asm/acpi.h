
#ifndef _ASM_X86_ACPI_H
#define _ASM_X86_ACPI_H

#include <acpi/pdc_intel.h>

#include <asm/numa.h>
#include <asm/processor.h>
#include <asm/mmu.h>
#include <asm/mpspec.h>

#define COMPILER_DEPENDENT_INT64   long long
#define COMPILER_DEPENDENT_UINT64  unsigned long long

#define ACPI_SYSTEM_XFACE
#define ACPI_EXTERNAL_XFACE
#define ACPI_INTERNAL_XFACE
#define ACPI_INTERNAL_VAR_XFACE

/* Asm macros */

#define ACPI_ASM_MACROS
#define BREAKPOINT3
#define ACPI_DISABLE_IRQS() local_irq_disable()
#define ACPI_ENABLE_IRQS()  local_irq_enable()
#define ACPI_FLUSH_CPU_CACHE()	wbinvd()

int __acpi_acquire_global_lock(unsigned int *lock);
int __acpi_release_global_lock(unsigned int *lock);

#define ACPI_ACQUIRE_GLOBAL_LOCK(facs, Acq) \
	((Acq) = __acpi_acquire_global_lock(&facs->global_lock))

#define ACPI_RELEASE_GLOBAL_LOCK(facs, Acq) \
	((Acq) = __acpi_release_global_lock(&facs->global_lock))

#define ACPI_DIV_64_BY_32(n_hi, n_lo, d32, q32, r32) \
	asm("divl %2;"				     \
	    : "=a"(q32), "=d"(r32)		     \
	    : "r"(d32),				     \
	     "0"(n_lo), "1"(n_hi))


#define ACPI_SHIFT_RIGHT_64(n_hi, n_lo) \
	asm("shrl   $1,%2	;"	\
	    "rcrl   $1,%3;"		\
	    : "=r"(n_hi), "=r"(n_lo)	\
	    : "0"(n_hi), "1"(n_lo))

#ifdef CONFIG_ACPI
extern int acpi_lapic;
extern int acpi_ioapic;
extern int acpi_noirq;
extern int acpi_strict;
extern int acpi_disabled;
extern int acpi_pci_disabled;
extern int acpi_skip_timer_override;
extern int acpi_use_timer_override;

extern u8 acpi_sci_flags;
extern int acpi_sci_override_gsi;
void acpi_pic_sci_set_trigger(unsigned int, u16);

static inline void disable_acpi(void)
{
	acpi_disabled = 1;
	acpi_pci_disabled = 1;
	acpi_noirq = 1;
}

extern int acpi_gsi_to_irq(u32 gsi, unsigned int *irq);

static inline void acpi_noirq_set(void) { acpi_noirq = 1; }
static inline void acpi_disable_pci(void)
{
	acpi_pci_disabled = 1;
	acpi_noirq_set();
}

/* routines for saving/restoring kernel state */
extern int acpi_save_state_mem(void);
extern void acpi_restore_state_mem(void);

extern unsigned long acpi_wakeup_address;

/* early initialization routine */
extern void acpi_reserve_wakeup_memory(void);

static inline unsigned int acpi_processor_cstate_check(unsigned int max_cstate)
{
	/*
	 * Early models (<=5) of AMD Opterons are not supposed to go into
	 * C2 state.
	 *
	 * Steppings 0x0A and later are good
	 */
	if (boot_cpu_data.x86 == 0x0F &&
	    boot_cpu_data.x86_vendor == X86_VENDOR_AMD &&
	    boot_cpu_data.x86_model <= 0x05 &&
	    boot_cpu_data.x86_mask < 0x0A)
		return 1;
	else if (boot_cpu_has(X86_FEATURE_AMDC1E))
		return 1;
	else
		return max_cstate;
}

static inline bool arch_has_acpi_pdc(void)
{
	struct cpuinfo_x86 *c = &cpu_data(0);
	return (c->x86_vendor == X86_VENDOR_INTEL ||
		c->x86_vendor == X86_VENDOR_CENTAUR);
}

static inline void arch_acpi_set_pdc_bits(u32 *buf)
{
	struct cpuinfo_x86 *c = &cpu_data(0);

	buf[2] |= ACPI_PDC_C_CAPABILITY_SMP;

	if (cpu_has(c, X86_FEATURE_EST))
		buf[2] |= ACPI_PDC_EST_CAPABILITY_SWSMP;

	if (cpu_has(c, X86_FEATURE_ACPI))
		buf[2] |= ACPI_PDC_T_FFH;

	/*
	 * If mwait/monitor is unsupported, C2/C3_FFH will be disabled
	 */
	if (!cpu_has(c, X86_FEATURE_MWAIT))
		buf[2] &= ~(ACPI_PDC_C_C2C3_FFH);
}

#else /* !CONFIG_ACPI */

#define acpi_lapic 0
#define acpi_ioapic 0
static inline void acpi_noirq_set(void) { }
static inline void acpi_disable_pci(void) { }
static inline void disable_acpi(void) { }

#endif /* !CONFIG_ACPI */

#define ARCH_HAS_POWER_INIT	1

struct bootnode;

#ifdef CONFIG_ACPI_NUMA
extern int acpi_numa;
extern int acpi_get_nodes(struct bootnode *physnodes);
extern int acpi_scan_nodes(unsigned long start, unsigned long end);
#define NR_NODE_MEMBLKS (MAX_NUMNODES*2)
extern void acpi_fake_nodes(const struct bootnode *fake_nodes,
				   int num_nodes);
#else
static inline void acpi_fake_nodes(const struct bootnode *fake_nodes,
				   int num_nodes)
{
}
#endif

#define acpi_unlazy_tlb(x)	leave_mm(x)

#endif /* _ASM_X86_ACPI_H */
