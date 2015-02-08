

#include <linux/linkage.h>
#include <asm/blackfin.h>
#include <asm/cache.h>
#include <asm/page.h>

.text

/* 05000443 - IFLUSH cannot be last instruction in hardware loop */
#if ANOMALY_05000443
# define BROK_FLUSH_INST "IFLUSH"
#else
# define BROK_FLUSH_INST "no anomaly! yeah!"
#endif

.macro do_flush flushins:req label

	R2 = -L1_CACHE_BYTES;

	/* start = (start & -L1_CACHE_BYTES) */
	R0 = R0 & R2;

	/* end = ((end - 1) & -L1_CACHE_BYTES) + L1_CACHE_BYTES; */
	R1 += -1;
	R1 = R1 & R2;
	R1 += L1_CACHE_BYTES;

	/* count = (end - start) >> L1_CACHE_SHIFT */
	R2 = R1 - R0;
	R2 >>= L1_CACHE_SHIFT;
	P1 = R2;

.ifnb \label
\label :
.endif
	P0 = R0;

	LSETUP (1f, 2f) LC1 = P1;
1:
.ifeqs "\flushins", BROK_FLUSH_INST
	\flushins [P0++];
2:	nop;
.else
2:	\flushins [P0++];
.endif

	RTS;
.endm

/* Invalidate all instruction cache lines assocoiated with this memory area */
ENTRY(_blackfin_icache_flush_range)
	P0 = R0;
	IFLUSH[P0];
	do_flush IFLUSH
ENDPROC(_blackfin_icache_flush_range)

ENTRY(_blackfin_dcache_invalidate_range)
	do_flush FLUSHINV
ENDPROC(_blackfin_dcache_invalidate_range)

/* Flush all data cache lines assocoiated with this memory area */
ENTRY(_blackfin_dcache_flush_range)
	do_flush FLUSH, .Ldfr
ENDPROC(_blackfin_dcache_flush_range)

ENTRY(_blackfin_dflush_page)
	P1 = 1 << (PAGE_SHIFT - L1_CACHE_SHIFT);
	jump .Ldfr;
ENDPROC(_blackfin_dflush_page)
