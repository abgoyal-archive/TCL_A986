

#include <linux/mman.h>
#include <linux/mm.h>
#include <linux/kernel.h>
#include <linux/ptrace.h>

#include <asm/system.h>
#include <asm/pgtable.h>

extern void die_if_kernel(char *, struct pt_regs *, long);

asmlinkage int do_page_fault(struct pt_regs *regs, unsigned long address,
			      unsigned long error_code)
{
#ifdef DEBUG
	printk(KERN_DEBUG "regs->sr=%#x, regs->pc=%#lx, address=%#lx, %ld\n",
		regs->sr, regs->pc, address, error_code);
#endif

	/*
	 * Oops. The kernel tried to access some bad page. We'll have to
	 * terminate things with extreme prejudice.
	 */
	if ((unsigned long) address < PAGE_SIZE)
		printk(KERN_ALERT "Unable to handle kernel NULL pointer dereference");
	else
		printk(KERN_ALERT "Unable to handle kernel access");
	printk(KERN_ALERT " at virtual address %08lx\n", address);
	die_if_kernel("Oops", regs, error_code);
	do_exit(SIGKILL);

	return 1;
}

