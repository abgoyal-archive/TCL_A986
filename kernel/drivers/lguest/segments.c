
#include "lg.h"


static bool ignored_gdt(unsigned int num)
{
	return (num == GDT_ENTRY_TSS
		|| num == GDT_ENTRY_LGUEST_CS
		|| num == GDT_ENTRY_LGUEST_DS
		|| num == GDT_ENTRY_DOUBLEFAULT_TSS);
}

static void fixup_gdt_table(struct lg_cpu *cpu, unsigned start, unsigned end)
{
	unsigned int i;

	for (i = start; i < end; i++) {
		/*
		 * We never copy these ones to real GDT, so we don't care what
		 * they say
		 */
		if (ignored_gdt(i))
			continue;

		/*
		 * Segment descriptors contain a privilege level: the Guest is
		 * sometimes careless and leaves this as 0, even though it's
		 * running at privilege level 1.  If so, we fix it here.
		 */
		if ((cpu->arch.gdt[i].b & 0x00006000) == 0)
			cpu->arch.gdt[i].b |= (GUEST_PL << 13);

		/*
		 * Each descriptor has an "accessed" bit.  If we don't set it
		 * now, the CPU will try to set it when the Guest first loads
		 * that entry into a segment register.  But the GDT isn't
		 * writable by the Guest, so bad things can happen.
		 */
		cpu->arch.gdt[i].b |= 0x00000100;
	}
}

void setup_default_gdt_entries(struct lguest_ro_state *state)
{
	struct desc_struct *gdt = state->guest_gdt;
	unsigned long tss = (unsigned long)&state->guest_tss;

	/* The Switcher segments are full 0-4G segments, privilege level 0 */
	gdt[GDT_ENTRY_LGUEST_CS] = FULL_EXEC_SEGMENT;
	gdt[GDT_ENTRY_LGUEST_DS] = FULL_SEGMENT;

	/*
	 * The TSS segment refers to the TSS entry for this particular CPU.
	 * Forgive the magic flags: the 0x8900 means the entry is Present, it's
	 * privilege level 0 Available 386 TSS system segment, and the 0x67
	 * means Saturn is eclipsed by Mercury in the twelfth house.
	 */
	gdt[GDT_ENTRY_TSS].a = 0x00000067 | (tss << 16);
	gdt[GDT_ENTRY_TSS].b = 0x00008900 | (tss & 0xFF000000)
		| ((tss >> 16) & 0x000000FF);
}

void setup_guest_gdt(struct lg_cpu *cpu)
{
	/*
	 * Start with full 0-4G segments...except the Guest is allowed to use
	 * them, so set the privilege level appropriately in the flags.
	 */
	cpu->arch.gdt[GDT_ENTRY_KERNEL_CS] = FULL_EXEC_SEGMENT;
	cpu->arch.gdt[GDT_ENTRY_KERNEL_DS] = FULL_SEGMENT;
	cpu->arch.gdt[GDT_ENTRY_KERNEL_CS].b |= (GUEST_PL << 13);
	cpu->arch.gdt[GDT_ENTRY_KERNEL_DS].b |= (GUEST_PL << 13);
}

void copy_gdt_tls(const struct lg_cpu *cpu, struct desc_struct *gdt)
{
	unsigned int i;

	for (i = GDT_ENTRY_TLS_MIN; i <= GDT_ENTRY_TLS_MAX; i++)
		gdt[i] = cpu->arch.gdt[i];
}

void copy_gdt(const struct lg_cpu *cpu, struct desc_struct *gdt)
{
	unsigned int i;

	/*
	 * The default entries from setup_default_gdt_entries() are not
	 * replaced.  See ignored_gdt() above.
	 */
	for (i = 0; i < GDT_ENTRIES; i++)
		if (!ignored_gdt(i))
			gdt[i] = cpu->arch.gdt[i];
}

void load_guest_gdt_entry(struct lg_cpu *cpu, u32 num, u32 lo, u32 hi)
{
	/*
	 * We assume the Guest has the same number of GDT entries as the
	 * Host, otherwise we'd have to dynamically allocate the Guest GDT.
	 */
	if (num >= ARRAY_SIZE(cpu->arch.gdt)) {
		kill_guest(cpu, "too many gdt entries %i", num);
		return;
	}

	/* Set it up, then fix it. */
	cpu->arch.gdt[num].a = lo;
	cpu->arch.gdt[num].b = hi;
	fixup_gdt_table(cpu, num, num+1);
	/*
	 * Mark that the GDT changed so the core knows it has to copy it again,
	 * even if the Guest is run on the same CPU.
	 */
	cpu->changed |= CHANGED_GDT;
}

void guest_load_tls(struct lg_cpu *cpu, unsigned long gtls)
{
	struct desc_struct *tls = &cpu->arch.gdt[GDT_ENTRY_TLS_MIN];

	__lgread(cpu, tls, gtls, sizeof(*tls)*GDT_ENTRY_TLS_ENTRIES);
	fixup_gdt_table(cpu, GDT_ENTRY_TLS_MIN, GDT_ENTRY_TLS_MAX+1);
	/* Note that just the TLS entries have changed. */
	cpu->changed |= CHANGED_GDT_TLS;
}

