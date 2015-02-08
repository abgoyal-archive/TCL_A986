
#ifndef _ASM_POWERPC_MODULE_H
#define _ASM_POWERPC_MODULE_H
#ifdef __KERNEL__


#include <linux/list.h>
#include <asm/bug.h>


#ifndef __powerpc64__

struct ppc_plt_entry {
	/* 16 byte jump instruction sequence (4 instructions) */
	unsigned int jump[4];
};
#endif	/* __powerpc64__ */


struct mod_arch_specific {
#ifdef __powerpc64__
	unsigned int stubs_section;	/* Index of stubs section in module */
	unsigned int toc_section;	/* What section is the TOC? */
#ifdef CONFIG_DYNAMIC_FTRACE
	unsigned long toc;
	unsigned long tramp;
#endif

#else /* powerpc64 */
	/* Indices of PLT sections within module. */
	unsigned int core_plt_section;
	unsigned int init_plt_section;
#ifdef CONFIG_DYNAMIC_FTRACE
	unsigned long tramp;
#endif
#endif /* powerpc64 */

	/* List of BUG addresses, source line numbers and filenames */
	struct list_head bug_list;
	struct bug_entry *bug_table;
	unsigned int num_bugs;
};


#ifdef __powerpc64__
#    define Elf_Shdr	Elf64_Shdr
#    define Elf_Sym	Elf64_Sym
#    define Elf_Ehdr	Elf64_Ehdr
#    ifdef MODULE
	asm(".section .stubs,\"ax\",@nobits; .align 3; .previous");
#    endif
#else
#    define Elf_Shdr	Elf32_Shdr
#    define Elf_Sym	Elf32_Sym
#    define Elf_Ehdr	Elf32_Ehdr
#    ifdef MODULE
	asm(".section .plt,\"ax\",@nobits; .align 3; .previous");
	asm(".section .init.plt,\"ax\",@nobits; .align 3; .previous");
#    endif	/* MODULE */
#endif

#ifdef CONFIG_DYNAMIC_FTRACE
#    ifdef MODULE
	asm(".section .ftrace.tramp,\"ax\",@nobits; .align 3; .previous");
#    endif	/* MODULE */
#endif


struct exception_table_entry;
void sort_ex_table(struct exception_table_entry *start,
		   struct exception_table_entry *finish);

#ifdef CONFIG_MODVERSIONS
#define ARCH_RELOCATES_KCRCTAB

extern const unsigned long reloc_start[];
#endif
#endif /* __KERNEL__ */
#endif	/* _ASM_POWERPC_MODULE_H */
