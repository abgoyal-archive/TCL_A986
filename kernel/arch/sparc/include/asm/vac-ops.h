
#ifndef _SPARC_VAC_OPS_H
#define _SPARC_VAC_OPS_H


#include <asm/sysen.h>
#include <asm/contregs.h>
#include <asm/asi.h>


/* Sun4c VAC Tags */
#define S4CVACTAG_CID      0x01c00000
#define S4CVACTAG_W        0x00200000
#define S4CVACTAG_P        0x00100000
#define S4CVACTAG_V        0x00080000
#define S4CVACTAG_TID      0x0000fffc

/* Sun4c VAC Virtual Address */
/* These aren't used, why bother? (Anton) */
#if 0
#define S4CVACVA_TID       0x3fff0000
#define S4CVACVA_LINE      0x0000fff0
#define S4CVACVA_BIL       0x0000000f
#endif


#define S4CVAC_BADBITS    0x0000f000

#define S4CVAC_BADALIAS(vaddr1, vaddr2) \
        ((((unsigned long) (vaddr1)) ^ ((unsigned long) (vaddr2))) & \
	 (S4CVAC_BADBITS))

struct sun4c_vac_props {
	unsigned int num_bytes;     /* Size of the cache */
	unsigned int do_hwflushes;  /* Hardware flushing available? */
	unsigned int linesize;      /* Size of each line in bytes */
	unsigned int log2lsize;     /* log2(linesize) */
	unsigned int on;            /* VAC is enabled */
};

extern struct sun4c_vac_props sun4c_vacinfo;

/* sun4c_enable_vac() enables the sun4c virtual address cache. */
static inline void sun4c_enable_vac(void)
{
	__asm__ __volatile__("lduba [%0] %1, %%g1\n\t"
			     "or    %%g1, %2, %%g1\n\t"
			     "stba  %%g1, [%0] %1\n\t"
			     : /* no outputs */
			     : "r" ((unsigned int) AC_SENABLE),
			     "i" (ASI_CONTROL), "i" (SENABLE_CACHE)
			     : "g1", "memory");
	sun4c_vacinfo.on = 1;
}

/* sun4c_disable_vac() disables the virtual address cache. */
static inline void sun4c_disable_vac(void)
{
	__asm__ __volatile__("lduba [%0] %1, %%g1\n\t"
			     "andn  %%g1, %2, %%g1\n\t"
			     "stba  %%g1, [%0] %1\n\t"
			     : /* no outputs */
			     : "r" ((unsigned int) AC_SENABLE),
			     "i" (ASI_CONTROL), "i" (SENABLE_CACHE)
			     : "g1", "memory");
	sun4c_vacinfo.on = 0;
}

#endif /* !(_SPARC_VAC_OPS_H) */
