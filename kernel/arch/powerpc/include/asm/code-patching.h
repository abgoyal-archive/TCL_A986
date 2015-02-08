
#ifndef _ASM_POWERPC_CODE_PATCHING_H
#define _ASM_POWERPC_CODE_PATCHING_H


#include <asm/types.h>
#include <asm/ppc-opcode.h>

#define BRANCH_SET_LINK	0x1
#define BRANCH_ABSOLUTE	0x2

unsigned int create_branch(const unsigned int *addr,
			   unsigned long target, int flags);
unsigned int create_cond_branch(const unsigned int *addr,
				unsigned long target, int flags);
void patch_branch(unsigned int *addr, unsigned long target, int flags);
void patch_instruction(unsigned int *addr, unsigned int instr);

int instr_is_relative_branch(unsigned int instr);
int instr_is_branch_to_addr(const unsigned int *instr, unsigned long addr);
unsigned long branch_target(const unsigned int *instr);
unsigned int translate_branch(const unsigned int *dest,
			      const unsigned int *src);

static inline unsigned long ppc_function_entry(void *func)
{
#ifdef CONFIG_PPC64
	/*
	 * On PPC64 the function pointer actually points to the function's
	 * descriptor. The first entry in the descriptor is the address
	 * of the function text.
	 */
	return ((func_descr_t *)func)->entry;
#else
	return (unsigned long)func;
#endif
}

#endif /* _ASM_POWERPC_CODE_PATCHING_H */
