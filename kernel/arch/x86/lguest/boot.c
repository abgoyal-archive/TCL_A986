

#include <linux/kernel.h>
#include <linux/start_kernel.h>
#include <linux/string.h>
#include <linux/console.h>
#include <linux/screen_info.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/lguest.h>
#include <linux/lguest_launcher.h>
#include <linux/virtio_console.h>
#include <linux/pm.h>
#include <asm/apic.h>
#include <asm/lguest.h>
#include <asm/paravirt.h>
#include <asm/param.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/desc.h>
#include <asm/setup.h>
#include <asm/e820.h>
#include <asm/mce.h>
#include <asm/io.h>
#include <asm/i387.h>
#include <asm/stackprotector.h>
#include <asm/reboot.h>		/* for struct machine_ops */


struct lguest_data lguest_data = {
	.hcall_status = { [0 ... LHCALL_RING_SIZE-1] = 0xFF },
	.noirq_start = (u32)lguest_noirq_start,
	.noirq_end = (u32)lguest_noirq_end,
	.kernel_address = PAGE_OFFSET,
	.blocked_interrupts = { 1 }, /* Block timer interrupts */
	.syscall_vec = SYSCALL_VECTOR,
};

static void async_hcall(unsigned long call, unsigned long arg1,
			unsigned long arg2, unsigned long arg3,
			unsigned long arg4)
{
	/* Note: This code assumes we're uniprocessor. */
	static unsigned int next_call;
	unsigned long flags;

	/*
	 * Disable interrupts if not already disabled: we don't want an
	 * interrupt handler making a hypercall while we're already doing
	 * one!
	 */
	local_irq_save(flags);
	if (lguest_data.hcall_status[next_call] != 0xFF) {
		/* Table full, so do normal hcall which will flush table. */
		hcall(call, arg1, arg2, arg3, arg4);
	} else {
		lguest_data.hcalls[next_call].arg0 = call;
		lguest_data.hcalls[next_call].arg1 = arg1;
		lguest_data.hcalls[next_call].arg2 = arg2;
		lguest_data.hcalls[next_call].arg3 = arg3;
		lguest_data.hcalls[next_call].arg4 = arg4;
		/* Arguments must all be written before we mark it to go */
		wmb();
		lguest_data.hcall_status[next_call] = 0;
		if (++next_call == LHCALL_RING_SIZE)
			next_call = 0;
	}
	local_irq_restore(flags);
}

static void lazy_hcall1(unsigned long call, unsigned long arg1)
{
	if (paravirt_get_lazy_mode() == PARAVIRT_LAZY_NONE)
		hcall(call, arg1, 0, 0, 0);
	else
		async_hcall(call, arg1, 0, 0, 0);
}

/* You can imagine what lazy_hcall2, 3 and 4 look like. :*/
static void lazy_hcall2(unsigned long call,
			unsigned long arg1,
			unsigned long arg2)
{
	if (paravirt_get_lazy_mode() == PARAVIRT_LAZY_NONE)
		hcall(call, arg1, arg2, 0, 0);
	else
		async_hcall(call, arg1, arg2, 0, 0);
}

static void lazy_hcall3(unsigned long call,
			unsigned long arg1,
			unsigned long arg2,
			unsigned long arg3)
{
	if (paravirt_get_lazy_mode() == PARAVIRT_LAZY_NONE)
		hcall(call, arg1, arg2, arg3, 0);
	else
		async_hcall(call, arg1, arg2, arg3, 0);
}

#ifdef CONFIG_X86_PAE
static void lazy_hcall4(unsigned long call,
			unsigned long arg1,
			unsigned long arg2,
			unsigned long arg3,
			unsigned long arg4)
{
	if (paravirt_get_lazy_mode() == PARAVIRT_LAZY_NONE)
		hcall(call, arg1, arg2, arg3, arg4);
	else
		async_hcall(call, arg1, arg2, arg3, arg4);
}
#endif

static void lguest_leave_lazy_mmu_mode(void)
{
	hcall(LHCALL_FLUSH_ASYNC, 0, 0, 0, 0);
	paravirt_leave_lazy_mmu();
}

static void lguest_end_context_switch(struct task_struct *next)
{
	hcall(LHCALL_FLUSH_ASYNC, 0, 0, 0, 0);
	paravirt_end_context_switch(next);
}


static unsigned long save_fl(void)
{
	return lguest_data.irq_enabled;
}

/* Interrupts go off... */
static void irq_disable(void)
{
	lguest_data.irq_enabled = 0;
}

PV_CALLEE_SAVE_REGS_THUNK(save_fl);
PV_CALLEE_SAVE_REGS_THUNK(irq_disable);
/*:*/

/* These are in i386_head.S */
extern void lg_irq_enable(void);
extern void lg_restore_fl(unsigned long flags);


static void lguest_write_idt_entry(gate_desc *dt,
				   int entrynum, const gate_desc *g)
{
	/*
	 * The gate_desc structure is 8 bytes long: we hand it to the Host in
	 * two 32-bit chunks.  The whole 32-bit kernel used to hand descriptors
	 * around like this; typesafety wasn't a big concern in Linux's early
	 * years.
	 */
	u32 *desc = (u32 *)g;
	/* Keep the local copy up to date. */
	native_write_idt_entry(dt, entrynum, g);
	/* Tell Host about this new entry. */
	hcall(LHCALL_LOAD_IDT_ENTRY, entrynum, desc[0], desc[1], 0);
}

static void lguest_load_idt(const struct desc_ptr *desc)
{
	unsigned int i;
	struct desc_struct *idt = (void *)desc->address;

	for (i = 0; i < (desc->size+1)/8; i++)
		hcall(LHCALL_LOAD_IDT_ENTRY, i, idt[i].a, idt[i].b, 0);
}

static void lguest_load_gdt(const struct desc_ptr *desc)
{
	unsigned int i;
	struct desc_struct *gdt = (void *)desc->address;

	for (i = 0; i < (desc->size+1)/8; i++)
		hcall(LHCALL_LOAD_GDT_ENTRY, i, gdt[i].a, gdt[i].b, 0);
}

static void lguest_write_gdt_entry(struct desc_struct *dt, int entrynum,
				   const void *desc, int type)
{
	native_write_gdt_entry(dt, entrynum, desc, type);
	/* Tell Host about this new entry. */
	hcall(LHCALL_LOAD_GDT_ENTRY, entrynum,
	      dt[entrynum].a, dt[entrynum].b, 0);
}

static void lguest_load_tls(struct thread_struct *t, unsigned int cpu)
{
	/*
	 * There's one problem which normal hardware doesn't have: the Host
	 * can't handle us removing entries we're currently using.  So we clear
	 * the GS register here: if it's needed it'll be reloaded anyway.
	 */
	lazy_load_gs(0);
	lazy_hcall2(LHCALL_LOAD_TLS, __pa(&t->tls_array), cpu);
}

static void lguest_set_ldt(const void *addr, unsigned entries)
{
}

static void lguest_load_tr_desc(void)
{
}

static void lguest_cpuid(unsigned int *ax, unsigned int *bx,
			 unsigned int *cx, unsigned int *dx)
{
	int function = *ax;

	native_cpuid(ax, bx, cx, dx);
	switch (function) {
	/*
	 * CPUID 0 gives the highest legal CPUID number (and the ID string).
	 * We futureproof our code a little by sticking to known CPUID values.
	 */
	case 0:
		if (*ax > 5)
			*ax = 5;
		break;

	/*
	 * CPUID 1 is a basic feature request.
	 *
	 * CX: we only allow kernel to see SSE3, CMPXCHG16B and SSSE3
	 * DX: SSE, SSE2, FXSR, MMX, CMOV, CMPXCHG8B, TSC, FPU and PAE.
	 */
	case 1:
		*cx &= 0x00002201;
		*dx &= 0x07808151;
		/*
		 * The Host can do a nice optimization if it knows that the
		 * kernel mappings (addresses above 0xC0000000 or whatever
		 * PAGE_OFFSET is set to) haven't changed.  But Linux calls
		 * flush_tlb_user() for both user and kernel mappings unless
		 * the Page Global Enable (PGE) feature bit is set.
		 */
		*dx |= 0x00002000;
		/*
		 * We also lie, and say we're family id 5.  6 or greater
		 * leads to a rdmsr in early_init_intel which we can't handle.
		 * Family ID is returned as bits 8-12 in ax.
		 */
		*ax &= 0xFFFFF0FF;
		*ax |= 0x00000500;
		break;
	/*
	 * 0x80000000 returns the highest Extended Function, so we futureproof
	 * like we do above by limiting it to known fields.
	 */
	case 0x80000000:
		if (*ax > 0x80000008)
			*ax = 0x80000008;
		break;

	/*
	 * PAE systems can mark pages as non-executable.  Linux calls this the
	 * NX bit.  Intel calls it XD (eXecute Disable), AMD EVP (Enhanced
	 * Virus Protection).  We just switch turn if off here, since we don't
	 * support it.
	 */
	case 0x80000001:
		*dx &= ~(1 << 20);
		break;
	}
}

static unsigned long current_cr0;
static void lguest_write_cr0(unsigned long val)
{
	lazy_hcall1(LHCALL_TS, val & X86_CR0_TS);
	current_cr0 = val;
}

static unsigned long lguest_read_cr0(void)
{
	return current_cr0;
}

static void lguest_clts(void)
{
	lazy_hcall1(LHCALL_TS, 0);
	current_cr0 &= ~X86_CR0_TS;
}

static unsigned long lguest_read_cr2(void)
{
	return lguest_data.cr2;
}

/* See lguest_set_pte() below. */
static bool cr3_changed = false;

static void lguest_write_cr3(unsigned long cr3)
{
	lguest_data.pgdir = cr3;
	lazy_hcall1(LHCALL_NEW_PGTABLE, cr3);
	cr3_changed = true;
}

static unsigned long lguest_read_cr3(void)
{
	return lguest_data.pgdir;
}

/* cr4 is used to enable and disable PGE, but we don't care. */
static unsigned long lguest_read_cr4(void)
{
	return 0;
}

static void lguest_write_cr4(unsigned long val)
{
}


static void lguest_pte_update(struct mm_struct *mm, unsigned long addr,
			       pte_t *ptep)
{
#ifdef CONFIG_X86_PAE
	/* PAE needs to hand a 64 bit page table entry, so it uses two args. */
	lazy_hcall4(LHCALL_SET_PTE, __pa(mm->pgd), addr,
		    ptep->pte_low, ptep->pte_high);
#else
	lazy_hcall3(LHCALL_SET_PTE, __pa(mm->pgd), addr, ptep->pte_low);
#endif
}

/* This is the "set and update" combo-meal-deal version. */
static void lguest_set_pte_at(struct mm_struct *mm, unsigned long addr,
			      pte_t *ptep, pte_t pteval)
{
	native_set_pte(ptep, pteval);
	lguest_pte_update(mm, addr, ptep);
}

#ifdef CONFIG_X86_PAE
static void lguest_set_pud(pud_t *pudp, pud_t pudval)
{
	native_set_pud(pudp, pudval);

	/* 32 bytes aligned pdpt address and the index. */
	lazy_hcall2(LHCALL_SET_PGD, __pa(pudp) & 0xFFFFFFE0,
		   (__pa(pudp) & 0x1F) / sizeof(pud_t));
}

static void lguest_set_pmd(pmd_t *pmdp, pmd_t pmdval)
{
	native_set_pmd(pmdp, pmdval);
	lazy_hcall2(LHCALL_SET_PMD, __pa(pmdp) & PAGE_MASK,
		   (__pa(pmdp) & (PAGE_SIZE - 1)) / sizeof(pmd_t));
}
#else

/* The Guest calls lguest_set_pmd to set a top-level entry when !PAE. */
static void lguest_set_pmd(pmd_t *pmdp, pmd_t pmdval)
{
	native_set_pmd(pmdp, pmdval);
	lazy_hcall2(LHCALL_SET_PGD, __pa(pmdp) & PAGE_MASK,
		   (__pa(pmdp) & (PAGE_SIZE - 1)) / sizeof(pmd_t));
}
#endif

static void lguest_set_pte(pte_t *ptep, pte_t pteval)
{
	native_set_pte(ptep, pteval);
	if (cr3_changed)
		lazy_hcall1(LHCALL_FLUSH_TLB, 1);
}

#ifdef CONFIG_X86_PAE
static void lguest_set_pte_atomic(pte_t *ptep, pte_t pte)
{
	native_set_pte_atomic(ptep, pte);
	if (cr3_changed)
		lazy_hcall1(LHCALL_FLUSH_TLB, 1);
}

static void lguest_pte_clear(struct mm_struct *mm, unsigned long addr,
			     pte_t *ptep)
{
	native_pte_clear(mm, addr, ptep);
	lguest_pte_update(mm, addr, ptep);
}

static void lguest_pmd_clear(pmd_t *pmdp)
{
	lguest_set_pmd(pmdp, __pmd(0));
}
#endif

static void lguest_flush_tlb_single(unsigned long addr)
{
	/* Simply set it to zero: if it was not, it will fault back in. */
	lazy_hcall3(LHCALL_SET_PTE, lguest_data.pgdir, addr, 0);
}

static void lguest_flush_tlb_user(void)
{
	lazy_hcall1(LHCALL_FLUSH_TLB, 0);
}

static void lguest_flush_tlb_kernel(void)
{
	lazy_hcall1(LHCALL_FLUSH_TLB, 1);
}

static void disable_lguest_irq(unsigned int irq)
{
	set_bit(irq, lguest_data.blocked_interrupts);
}

static void enable_lguest_irq(unsigned int irq)
{
	clear_bit(irq, lguest_data.blocked_interrupts);
}

/* This structure describes the lguest IRQ controller. */
static struct irq_chip lguest_irq_controller = {
	.name		= "lguest",
	.mask		= disable_lguest_irq,
	.mask_ack	= disable_lguest_irq,
	.unmask		= enable_lguest_irq,
};

static void __init lguest_init_IRQ(void)
{
	unsigned int i;

	for (i = FIRST_EXTERNAL_VECTOR; i < NR_VECTORS; i++) {
		/* Some systems map "vectors" to interrupts weirdly.  Not us! */
		__get_cpu_var(vector_irq)[i] = i - FIRST_EXTERNAL_VECTOR;
		if (i != SYSCALL_VECTOR)
			set_intr_gate(i, interrupt[i - FIRST_EXTERNAL_VECTOR]);
	}

	/*
	 * This call is required to set up for 4k stacks, where we have
	 * separate stacks for hard and soft interrupts.
	 */
	irq_ctx_init(smp_processor_id());
}

void lguest_setup_irq(unsigned int irq)
{
	irq_to_desc_alloc_node(irq, 0);
	set_irq_chip_and_handler_name(irq, &lguest_irq_controller,
				      handle_level_irq, "level");
}

static unsigned long lguest_get_wallclock(void)
{
	return lguest_data.time.tv_sec;
}

static unsigned long lguest_tsc_khz(void)
{
	return lguest_data.tsc_khz;
}

static cycle_t lguest_clock_read(struct clocksource *cs)
{
	unsigned long sec, nsec;

	/*
	 * Since the time is in two parts (seconds and nanoseconds), we risk
	 * reading it just as it's changing from 99 & 0.999999999 to 100 and 0,
	 * and getting 99 and 0.  As Linux tends to come apart under the stress
	 * of time travel, we must be careful:
	 */
	do {
		/* First we read the seconds part. */
		sec = lguest_data.time.tv_sec;
		/*
		 * This read memory barrier tells the compiler and the CPU that
		 * this can't be reordered: we have to complete the above
		 * before going on.
		 */
		rmb();
		/* Now we read the nanoseconds part. */
		nsec = lguest_data.time.tv_nsec;
		/* Make sure we've done that. */
		rmb();
		/* Now if the seconds part has changed, try again. */
	} while (unlikely(lguest_data.time.tv_sec != sec));

	/* Our lguest clock is in real nanoseconds. */
	return sec*1000000000ULL + nsec;
}

/* This is the fallback clocksource: lower priority than the TSC clocksource. */
static struct clocksource lguest_clock = {
	.name		= "lguest",
	.rating		= 200,
	.read		= lguest_clock_read,
	.mask		= CLOCKSOURCE_MASK(64),
	.mult		= 1 << 22,
	.shift		= 22,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static int lguest_clockevent_set_next_event(unsigned long delta,
                                           struct clock_event_device *evt)
{
	/* FIXME: I don't think this can ever happen, but James tells me he had
	 * to put this code in.  Maybe we should remove it now.  Anyone? */
	if (delta < LG_CLOCK_MIN_DELTA) {
		if (printk_ratelimit())
			printk(KERN_DEBUG "%s: small delta %lu ns\n",
			       __func__, delta);
		return -ETIME;
	}

	/* Please wake us this far in the future. */
	hcall(LHCALL_SET_CLOCKEVENT, delta, 0, 0, 0);
	return 0;
}

static void lguest_clockevent_set_mode(enum clock_event_mode mode,
                                      struct clock_event_device *evt)
{
	switch (mode) {
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		/* A 0 argument shuts the clock down. */
		hcall(LHCALL_SET_CLOCKEVENT, 0, 0, 0, 0);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		/* This is what we expect. */
		break;
	case CLOCK_EVT_MODE_PERIODIC:
		BUG();
	case CLOCK_EVT_MODE_RESUME:
		break;
	}
}

/* This describes our primitive timer chip. */
static struct clock_event_device lguest_clockevent = {
	.name                   = "lguest",
	.features               = CLOCK_EVT_FEAT_ONESHOT,
	.set_next_event         = lguest_clockevent_set_next_event,
	.set_mode               = lguest_clockevent_set_mode,
	.rating                 = INT_MAX,
	.mult                   = 1,
	.shift                  = 0,
	.min_delta_ns           = LG_CLOCK_MIN_DELTA,
	.max_delta_ns           = LG_CLOCK_MAX_DELTA,
};

static void lguest_time_irq(unsigned int irq, struct irq_desc *desc)
{
	unsigned long flags;

	/* Don't interrupt us while this is running. */
	local_irq_save(flags);
	lguest_clockevent.event_handler(&lguest_clockevent);
	local_irq_restore(flags);
}

static void lguest_time_init(void)
{
	/* Set up the timer interrupt (0) to go to our simple timer routine */
	set_irq_handler(0, lguest_time_irq);

	clocksource_register(&lguest_clock);

	/* We can't set cpumask in the initializer: damn C limitations!  Set it
	 * here and register our timer device. */
	lguest_clockevent.cpumask = cpumask_of(0);
	clockevents_register_device(&lguest_clockevent);

	/* Finally, we unblock the timer interrupt. */
	enable_lguest_irq(0);
}


static void lguest_load_sp0(struct tss_struct *tss,
			    struct thread_struct *thread)
{
	lazy_hcall3(LHCALL_SET_STACK, __KERNEL_DS | 0x1, thread->sp0,
		   THREAD_SIZE / PAGE_SIZE);
}

/* Let's just say, I wouldn't do debugging under a Guest. */
static void lguest_set_debugreg(int regno, unsigned long value)
{
	/* FIXME: Implement */
}

static void lguest_wbinvd(void)
{
}

#ifdef CONFIG_X86_LOCAL_APIC
static void lguest_apic_write(u32 reg, u32 v)
{
}

static u32 lguest_apic_read(u32 reg)
{
	return 0;
}

static u64 lguest_apic_icr_read(void)
{
	return 0;
}

static void lguest_apic_icr_write(u32 low, u32 id)
{
	/* Warn to see if there's any stray references */
	WARN_ON(1);
}

static void lguest_apic_wait_icr_idle(void)
{
	return;
}

static u32 lguest_apic_safe_wait_icr_idle(void)
{
	return 0;
}

static void set_lguest_basic_apic_ops(void)
{
	apic->read = lguest_apic_read;
	apic->write = lguest_apic_write;
	apic->icr_read = lguest_apic_icr_read;
	apic->icr_write = lguest_apic_icr_write;
	apic->wait_icr_idle = lguest_apic_wait_icr_idle;
	apic->safe_wait_icr_idle = lguest_apic_safe_wait_icr_idle;
};
#endif

/* STOP!  Until an interrupt comes in. */
static void lguest_safe_halt(void)
{
	hcall(LHCALL_HALT, 0, 0, 0, 0);
}

static void lguest_power_off(void)
{
	hcall(LHCALL_SHUTDOWN, __pa("Power down"),
	      LGUEST_SHUTDOWN_POWEROFF, 0, 0);
}

static int lguest_panic(struct notifier_block *nb, unsigned long l, void *p)
{
	hcall(LHCALL_SHUTDOWN, __pa(p), LGUEST_SHUTDOWN_POWEROFF, 0, 0);
	/* The hcall won't return, but to keep gcc happy, we're "done". */
	return NOTIFY_DONE;
}

static struct notifier_block paniced = {
	.notifier_call = lguest_panic
};

/* Setting up memory is fairly easy. */
static __init char *lguest_memory_setup(void)
{
	/*
	 *The Linux bootloader header contains an "e820" memory map: the
	 * Launcher populated the first entry with our memory limit.
	 */
	e820_add_region(boot_params.e820_map[0].addr,
			  boot_params.e820_map[0].size,
			  boot_params.e820_map[0].type);

	/* This string is for the boot messages. */
	return "LGUEST";
}

static __init int early_put_chars(u32 vtermno, const char *buf, int count)
{
	char scratch[17];
	unsigned int len = count;

	/* We use a nul-terminated string, so we make a copy.  Icky, huh? */
	if (len > sizeof(scratch) - 1)
		len = sizeof(scratch) - 1;
	scratch[len] = '\0';
	memcpy(scratch, buf, len);
	hcall(LHCALL_NOTIFY, __pa(scratch), 0, 0, 0);

	/* This routine returns the number of bytes actually written. */
	return len;
}

static void lguest_restart(char *reason)
{
	hcall(LHCALL_SHUTDOWN, __pa(reason), LGUEST_SHUTDOWN_RESTART, 0, 0);
}


/*G:060 We construct a table from the assembler templates: */
static const struct lguest_insns
{
	const char *start, *end;
} lguest_insns[] = {
	[PARAVIRT_PATCH(pv_irq_ops.irq_disable)] = { lgstart_cli, lgend_cli },
	[PARAVIRT_PATCH(pv_irq_ops.save_fl)] = { lgstart_pushf, lgend_pushf },
};

static unsigned lguest_patch(u8 type, u16 clobber, void *ibuf,
			     unsigned long addr, unsigned len)
{
	unsigned int insn_len;

	/* Don't do anything special if we don't have a replacement */
	if (type >= ARRAY_SIZE(lguest_insns) || !lguest_insns[type].start)
		return paravirt_patch_default(type, clobber, ibuf, addr, len);

	insn_len = lguest_insns[type].end - lguest_insns[type].start;

	/* Similarly if it can't fit (doesn't happen, but let's be thorough). */
	if (len < insn_len)
		return paravirt_patch_default(type, clobber, ibuf, addr, len);

	/* Copy in our instructions. */
	memcpy(ibuf, lguest_insns[type].start, insn_len);
	return insn_len;
}

__init void lguest_init(void)
{
	/* We're under lguest. */
	pv_info.name = "lguest";
	/* Paravirt is enabled. */
	pv_info.paravirt_enabled = 1;
	/* We're running at privilege level 1, not 0 as normal. */
	pv_info.kernel_rpl = 1;
	/* Everyone except Xen runs with this set. */
	pv_info.shared_kernel_pmd = 1;

	/*
	 * We set up all the lguest overrides for sensitive operations.  These
	 * are detailed with the operations themselves.
	 */

	/* Interrupt-related operations */
	pv_irq_ops.save_fl = PV_CALLEE_SAVE(save_fl);
	pv_irq_ops.restore_fl = __PV_IS_CALLEE_SAVE(lg_restore_fl);
	pv_irq_ops.irq_disable = PV_CALLEE_SAVE(irq_disable);
	pv_irq_ops.irq_enable = __PV_IS_CALLEE_SAVE(lg_irq_enable);
	pv_irq_ops.safe_halt = lguest_safe_halt;

	/* Setup operations */
	pv_init_ops.patch = lguest_patch;

	/* Intercepts of various CPU instructions */
	pv_cpu_ops.load_gdt = lguest_load_gdt;
	pv_cpu_ops.cpuid = lguest_cpuid;
	pv_cpu_ops.load_idt = lguest_load_idt;
	pv_cpu_ops.iret = lguest_iret;
	pv_cpu_ops.nmi_return = lguest_iret;
	pv_cpu_ops.load_sp0 = lguest_load_sp0;
	pv_cpu_ops.load_tr_desc = lguest_load_tr_desc;
	pv_cpu_ops.set_ldt = lguest_set_ldt;
	pv_cpu_ops.load_tls = lguest_load_tls;
	pv_cpu_ops.set_debugreg = lguest_set_debugreg;
	pv_cpu_ops.clts = lguest_clts;
	pv_cpu_ops.read_cr0 = lguest_read_cr0;
	pv_cpu_ops.write_cr0 = lguest_write_cr0;
	pv_cpu_ops.read_cr4 = lguest_read_cr4;
	pv_cpu_ops.write_cr4 = lguest_write_cr4;
	pv_cpu_ops.write_gdt_entry = lguest_write_gdt_entry;
	pv_cpu_ops.write_idt_entry = lguest_write_idt_entry;
	pv_cpu_ops.wbinvd = lguest_wbinvd;
	pv_cpu_ops.start_context_switch = paravirt_start_context_switch;
	pv_cpu_ops.end_context_switch = lguest_end_context_switch;

	/* Pagetable management */
	pv_mmu_ops.write_cr3 = lguest_write_cr3;
	pv_mmu_ops.flush_tlb_user = lguest_flush_tlb_user;
	pv_mmu_ops.flush_tlb_single = lguest_flush_tlb_single;
	pv_mmu_ops.flush_tlb_kernel = lguest_flush_tlb_kernel;
	pv_mmu_ops.set_pte = lguest_set_pte;
	pv_mmu_ops.set_pte_at = lguest_set_pte_at;
	pv_mmu_ops.set_pmd = lguest_set_pmd;
#ifdef CONFIG_X86_PAE
	pv_mmu_ops.set_pte_atomic = lguest_set_pte_atomic;
	pv_mmu_ops.pte_clear = lguest_pte_clear;
	pv_mmu_ops.pmd_clear = lguest_pmd_clear;
	pv_mmu_ops.set_pud = lguest_set_pud;
#endif
	pv_mmu_ops.read_cr2 = lguest_read_cr2;
	pv_mmu_ops.read_cr3 = lguest_read_cr3;
	pv_mmu_ops.lazy_mode.enter = paravirt_enter_lazy_mmu;
	pv_mmu_ops.lazy_mode.leave = lguest_leave_lazy_mmu_mode;
	pv_mmu_ops.pte_update = lguest_pte_update;
	pv_mmu_ops.pte_update_defer = lguest_pte_update;

#ifdef CONFIG_X86_LOCAL_APIC
	/* APIC read/write intercepts */
	set_lguest_basic_apic_ops();
#endif

	x86_init.resources.memory_setup = lguest_memory_setup;
	x86_init.irqs.intr_init = lguest_init_IRQ;
	x86_init.timers.timer_init = lguest_time_init;
	x86_platform.calibrate_tsc = lguest_tsc_khz;
	x86_platform.get_wallclock =  lguest_get_wallclock;

	/*
	 * Now is a good time to look at the implementations of these functions
	 * before returning to the rest of lguest_init().
	 */

	/*G:070
	 * Now we've seen all the paravirt_ops, we return to
	 * lguest_init() where the rest of the fairly chaotic boot setup
	 * occurs.
	 */

	/*
	 * The stack protector is a weird thing where gcc places a canary
	 * value on the stack and then checks it on return.  This file is
	 * compiled with -fno-stack-protector it, so we got this far without
	 * problems.  The value of the canary is kept at offset 20 from the
	 * %gs register, so we need to set that up before calling C functions
	 * in other files.
	 */
	setup_stack_canary_segment(0);

	/*
	 * We could just call load_stack_canary_segment(), but we might as well
	 * call switch_to_new_gdt() which loads the whole table and sets up the
	 * per-cpu segment descriptor register %fs as well.
	 */
	switch_to_new_gdt(0);

	/* We actually boot with all memory mapped, but let's say 128MB. */
	max_pfn_mapped = (128*1024*1024) >> PAGE_SHIFT;

	/*
	 * The Host<->Guest Switcher lives at the top of our address space, and
	 * the Host told us how big it is when we made LGUEST_INIT hypercall:
	 * it put the answer in lguest_data.reserve_mem
	 */
	reserve_top_address(lguest_data.reserve_mem);

	/*
	 * If we don't initialize the lock dependency checker now, it crashes
	 * atomic_notifier_chain_register, then paravirt_disable_iospace.
	 */
	lockdep_init();

	/* Hook in our special panic hypercall code. */
	atomic_notifier_chain_register(&panic_notifier_list, &paniced);

	/*
	 * The IDE code spends about 3 seconds probing for disks: if we reserve
	 * all the I/O ports up front it can't get them and so doesn't probe.
	 * Other device drivers are similar (but less severe).  This cuts the
	 * kernel boot time on my machine from 4.1 seconds to 0.45 seconds.
	 */
	paravirt_disable_iospace();

	/*
	 * This is messy CPU setup stuff which the native boot code does before
	 * start_kernel, so we have to do, too:
	 */
	cpu_detect(&new_cpu_data);
	/* head.S usually sets up the first capability word, so do it here. */
	new_cpu_data.x86_capability[0] = cpuid_edx(1);

	/* Math is always hard! */
	new_cpu_data.hard_math = 1;

	/* We don't have features.  We have puppies!  Puppies! */
#ifdef CONFIG_X86_MCE
	mce_disabled = 1;
#endif
#ifdef CONFIG_ACPI
	acpi_disabled = 1;
#endif

	/*
	 * We set the preferred console to "hvc".  This is the "hypervisor
	 * virtual console" driver written by the PowerPC people, which we also
	 * adapted for lguest's use.
	 */
	add_preferred_console("hvc", 0, NULL);

	/* Register our very early console. */
	virtio_cons_early_init(early_put_chars);

	/*
	 * Last of all, we set the power management poweroff hook to point to
	 * the Guest routine to power off, and the reboot hook to our restart
	 * routine.
	 */
	pm_power_off = lguest_power_off;
	machine_ops.restart = lguest_restart;

	/*
	 * Now we're set up, call i386_start_kernel() in head32.c and we proceed
	 * to boot as normal.  It never returns.
	 */
	i386_start_kernel();
}
