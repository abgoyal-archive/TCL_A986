
#ifndef	_ASM_CACHECTL
#define	_ASM_CACHECTL

#define	ICACHE	(1<<0)		/* flush instruction cache        */
#define	DCACHE	(1<<1)		/* writeback and flush data cache */
#define	BCACHE	(ICACHE|DCACHE)	/* flush both caches              */

#define CACHEABLE	0	/* make pages cacheable */
#define UNCACHEABLE	1	/* make pages uncacheable */

#endif	/* _ASM_CACHECTL */
