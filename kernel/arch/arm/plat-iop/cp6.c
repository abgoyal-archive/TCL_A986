
#include <linux/init.h>
#include <asm/traps.h>

static int cp6_trap(struct pt_regs *regs, unsigned int instr)
{
	u32 temp;

        /* enable cp6 access */
        asm volatile (
		"mrc	p15, 0, %0, c15, c1, 0\n\t"
		"orr	%0, %0, #(1 << 6)\n\t"
		"mcr	p15, 0, %0, c15, c1, 0\n\t"
		: "=r"(temp));

	return 0;
}

static struct undef_hook cp6_hook = {
	.instr_mask     = 0x0f000ff0,
	.instr_val      = 0x0e000610,
	.cpsr_mask      = MODE_MASK,
	.cpsr_val       = SVC_MODE,
	.fn             = cp6_trap,
};

void __init iop_init_cp6_handler(void)
{
	register_undef_hook(&cp6_hook);
}
