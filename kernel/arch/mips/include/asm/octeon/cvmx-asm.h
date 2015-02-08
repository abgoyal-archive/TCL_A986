

#ifndef __CVMX_ASM_H__
#define __CVMX_ASM_H__

#include "octeon-model.h"

/* other useful stuff */
#define CVMX_SYNC asm volatile ("sync" : : : "memory")
/* String version of SYNCW macro for using in inline asm constructs */
#define CVMX_SYNCW_STR "syncw\nsyncw\n"
#ifdef __OCTEON__

/* Deprecated, will be removed in future release */
#define CVMX_SYNCIO asm volatile ("nop")

#define CVMX_SYNCIOBDMA asm volatile ("synciobdma" : : : "memory")

/* Deprecated, will be removed in future release */
#define CVMX_SYNCIOALL asm volatile ("nop")

#define CVMX_SYNCW asm volatile ("syncw\n\tsyncw" : : : "memory")

#define CVMX_SYNCWS CVMX_SYNCW
#define CVMX_SYNCS  CVMX_SYNC
#define CVMX_SYNCWS_STR CVMX_SYNCW_STR
#else
/* Deprecated, will be removed in future release */
#define CVMX_SYNCIO asm volatile ("nop")

#define CVMX_SYNCIOBDMA asm volatile ("sync" : : : "memory")

/* Deprecated, will be removed in future release */
#define CVMX_SYNCIOALL asm volatile ("nop")

#define CVMX_SYNCW asm volatile ("sync" : : : "memory")
#define CVMX_SYNCWS CVMX_SYNCW
#define CVMX_SYNCS  CVMX_SYNC
#define CVMX_SYNCWS_STR CVMX_SYNCW_STR
#endif

#define CVMX_PREPARE_FOR_STORE(address, offset) \
	asm volatile ("pref 30, " CVMX_TMP_STR(offset) "(%[rbase])" : : \
	[rbase] "d" (address))
#define CVMX_DONT_WRITE_BACK(address, offset) \
	asm volatile ("pref 29, " CVMX_TMP_STR(offset) "(%[rbase])" : : \
	[rbase] "d" (address))

/* flush stores, invalidate entire icache */
#define CVMX_ICACHE_INVALIDATE \
	{ CVMX_SYNC; asm volatile ("synci 0($0)" : : ); }

/* flush stores, invalidate entire icache */
#define CVMX_ICACHE_INVALIDATE2 \
	{ CVMX_SYNC; asm volatile ("cache 0, 0($0)" : : ); }

/* complete prefetches, invalidate entire dcache */
#define CVMX_DCACHE_INVALIDATE \
	{ CVMX_SYNC; asm volatile ("cache 9, 0($0)" : : ); }


#define CVMX_POP(result, input) \
	asm ("pop %[rd],%[rs]" : [rd] "=d" (result) : [rs] "d" (input))
#define CVMX_DPOP(result, input) \
	asm ("dpop %[rd],%[rs]" : [rd] "=d" (result) : [rs] "d" (input))

/* some new cop0-like stuff */
#define CVMX_RDHWR(result, regstr) \
	asm volatile ("rdhwr %[rt],$" CVMX_TMP_STR(regstr) : [rt] "=d" (result))
#define CVMX_RDHWRNV(result, regstr) \
	asm ("rdhwr %[rt],$" CVMX_TMP_STR(regstr) : [rt] "=d" (result))
#endif /* __CVMX_ASM_H__ */
