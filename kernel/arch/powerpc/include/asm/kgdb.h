
#ifdef __KERNEL__
#ifndef __POWERPC_KGDB_H__
#define __POWERPC_KGDB_H__

#ifndef __ASSEMBLY__

#define BREAK_INSTR_SIZE	4
#define BUFMAX			((NUMREGBYTES * 2) + 512)
#define OUTBUFMAX		((NUMREGBYTES * 2) + 512)
static inline void arch_kgdb_breakpoint(void)
{
	asm(".long 0x7d821008"); /* twge r2, r2 */
}
#define CACHE_FLUSH_IS_SAFE	1

#ifdef CONFIG_PPC64
#define NUMREGBYTES		((68 * 8) + (3 * 4))
#define NUMCRITREGBYTES		184
#else /* CONFIG_PPC32 */
#ifndef CONFIG_E500
#define MAXREG			(PT_FPSCR+1)
#else
/* 32 GPRs (8 bytes), nip, msr, ccr, link, ctr, xer, acc (8 bytes), spefscr*/
#define MAXREG                 ((32*2)+6+2+1)
#endif
#define NUMREGBYTES		(MAXREG * sizeof(int))
/* CR/LR, R1, R2, R13-R31 inclusive. */
#define NUMCRITREGBYTES		(23 * sizeof(int))
#endif /* 32/64 */
#endif /* !(__ASSEMBLY__) */
#endif /* !__POWERPC_KGDB_H__ */
#endif /* __KERNEL__ */
