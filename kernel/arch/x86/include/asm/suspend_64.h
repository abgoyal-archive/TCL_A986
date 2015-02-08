
#ifndef _ASM_X86_SUSPEND_64_H
#define _ASM_X86_SUSPEND_64_H

#include <asm/desc.h>
#include <asm/i387.h>

static inline int arch_prepare_suspend(void)
{
	return 0;
}

struct saved_context {
	struct pt_regs regs;
	u16 ds, es, fs, gs, ss;
	unsigned long gs_base, gs_kernel_base, fs_base;
	unsigned long cr0, cr2, cr3, cr4, cr8;
	u64 misc_enable;
	bool misc_enable_saved;
	unsigned long efer;
	u16 gdt_pad;
	u16 gdt_limit;
	unsigned long gdt_base;
	u16 idt_pad;
	u16 idt_limit;
	unsigned long idt_base;
	u16 ldt;
	u16 tss;
	unsigned long tr;
	unsigned long safety;
	unsigned long return_address;
} __attribute__((packed));

#define loaddebug(thread,register) \
	set_debugreg((thread)->debugreg##register, register)

/* routines for saving/restoring kernel state */
extern int acpi_save_state_mem(void);
extern char core_restore_code;
extern char restore_registers;

#endif /* _ASM_X86_SUSPEND_64_H */
