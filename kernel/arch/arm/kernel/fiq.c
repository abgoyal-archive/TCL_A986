
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/seq_file.h>

#include <asm/cacheflush.h>
#include <asm/fiq.h>
#include <asm/irq.h>
#include <asm/system.h>

static unsigned long no_fiq_insn;

static int fiq_def_op(void *ref, int relinquish)
{
	if (!relinquish)
		set_fiq_handler(&no_fiq_insn, sizeof(no_fiq_insn));

	return 0;
}

static struct fiq_handler default_owner = {
	.name	= "default",
	.fiq_op = fiq_def_op,
};

static struct fiq_handler *current_fiq = &default_owner;

int show_fiq_list(struct seq_file *p, void *v)
{
	if (current_fiq != &default_owner)
		seq_printf(p, "FIQ:              %s\n", current_fiq->name);

	return 0;
}

void set_fiq_handler(void *start, unsigned int length)
{
	memcpy((void *)0xffff001c, start, length);
	flush_icache_range(0xffff001c, 0xffff001c + length);
	if (!vectors_high())
		flush_icache_range(0x1c, 0x1c + length);
}

void __naked set_fiq_regs(struct pt_regs *regs)
{
	register unsigned long tmp;
	asm volatile (
	"mov	ip, sp\n\
	stmfd	sp!, {fp, ip, lr, pc}\n\
	sub	fp, ip, #4\n\
	mrs	%0, cpsr\n\
	msr	cpsr_c, %2	@ select FIQ mode\n\
	mov	r0, r0\n\
	ldmia	%1, {r8 - r14}\n\
	msr	cpsr_c, %0	@ return to SVC mode\n\
	mov	r0, r0\n\
	ldmfd	sp, {fp, sp, pc}"
	: "=&r" (tmp)
	: "r" (&regs->ARM_r8), "I" (PSR_I_BIT | PSR_F_BIT | FIQ_MODE));
}

void __naked get_fiq_regs(struct pt_regs *regs)
{
	register unsigned long tmp;
	asm volatile (
	"mov	ip, sp\n\
	stmfd	sp!, {fp, ip, lr, pc}\n\
	sub	fp, ip, #4\n\
	mrs	%0, cpsr\n\
	msr	cpsr_c, %2	@ select FIQ mode\n\
	mov	r0, r0\n\
	stmia	%1, {r8 - r14}\n\
	msr	cpsr_c, %0	@ return to SVC mode\n\
	mov	r0, r0\n\
	ldmfd	sp, {fp, sp, pc}"
	: "=&r" (tmp)
	: "r" (&regs->ARM_r8), "I" (PSR_I_BIT | PSR_F_BIT | FIQ_MODE));
}

int claim_fiq(struct fiq_handler *f)
{
	int ret = 0;

	if (current_fiq) {
		ret = -EBUSY;

		if (current_fiq->fiq_op != NULL)
			ret = current_fiq->fiq_op(current_fiq->dev_id, 1);
	}

	if (!ret) {
		f->next = current_fiq;
		current_fiq = f;
	}

	return ret;
}

void release_fiq(struct fiq_handler *f)
{
	if (current_fiq != f) {
		printk(KERN_ERR "%s FIQ trying to release %s FIQ\n",
		       f->name, current_fiq->name);
		dump_stack();
		return;
	}

	do
		current_fiq = current_fiq->next;
	while (current_fiq->fiq_op(current_fiq->dev_id, 0));
}

void enable_fiq(int fiq)
{
	enable_irq(fiq + FIQ_START);
}

void disable_fiq(int fiq)
{
	disable_irq(fiq + FIQ_START);
}

EXPORT_SYMBOL(set_fiq_handler);
EXPORT_SYMBOL(set_fiq_regs);
EXPORT_SYMBOL(get_fiq_regs);
EXPORT_SYMBOL(claim_fiq);
EXPORT_SYMBOL(release_fiq);
EXPORT_SYMBOL(enable_fiq);
EXPORT_SYMBOL(disable_fiq);

void __init init_FIQ(void)
{
	no_fiq_insn = *(unsigned long *)0xffff001c;
}
