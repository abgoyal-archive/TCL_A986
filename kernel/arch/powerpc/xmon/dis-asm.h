
#ifndef _POWERPC_XMON_DIS_ASM_H
#define _POWERPC_XMON_DIS_ASM_H

extern void print_address (unsigned long memaddr);

#ifdef CONFIG_XMON_DISASSEMBLY
extern int print_insn_powerpc(unsigned long insn, unsigned long memaddr);
extern int print_insn_spu(unsigned long insn, unsigned long memaddr);
#else
static inline int print_insn_powerpc(unsigned long insn, unsigned long memaddr)
{
	printf("%.8x", insn);
	return 0;
}

static inline int print_insn_spu(unsigned long insn, unsigned long memaddr)
{
	printf("%.8x", insn);
	return 0;
}
#endif

#endif /* _POWERPC_XMON_DIS_ASM_H */
