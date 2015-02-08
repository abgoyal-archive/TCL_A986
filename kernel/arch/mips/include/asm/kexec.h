

#ifndef _MIPS_KEXEC
# define _MIPS_KEXEC

/* Maximum physical address we can use pages from */
#define KEXEC_SOURCE_MEMORY_LIMIT (0x20000000)
/* Maximum address we can reach in physical address mode */
#define KEXEC_DESTINATION_MEMORY_LIMIT (0x20000000)
 /* Maximum address we can use for the control code buffer */
#define KEXEC_CONTROL_MEMORY_LIMIT (0x20000000)

#define KEXEC_CONTROL_PAGE_SIZE 4096

/* The native architecture */
#define KEXEC_ARCH KEXEC_ARCH_MIPS

static inline void crash_setup_regs(struct pt_regs *newregs,
				    struct pt_regs *oldregs)
{
	/* Dummy implementation for now */
}

#endif /* !_MIPS_KEXEC */
