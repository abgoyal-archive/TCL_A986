

#ifndef _ASM_X86_SRAT_H
#define _ASM_X86_SRAT_H

#ifdef CONFIG_ACPI_NUMA
extern int get_memcfg_from_srat(void);
#else
static inline int get_memcfg_from_srat(void)
{
	return 0;
}
#endif

#endif /* _ASM_X86_SRAT_H */
