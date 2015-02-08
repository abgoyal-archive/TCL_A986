

#ifndef _XTENSA_MODULE_H
#define _XTENSA_MODULE_H

struct mod_arch_specific
{
	/* No special elements, yet. */
};

#define MODULE_ARCH_VERMAGIC "xtensa-" __stringify(XCHAL_CORE_ID) " "

#define Elf_Shdr Elf32_Shdr
#define Elf_Sym Elf32_Sym
#define Elf_Ehdr Elf32_Ehdr

#endif	/* _XTENSA_MODULE_H */
