

#ifndef _ASM_MICROBLAZE_CACHE_H
#define _ASM_MICROBLAZE_CACHE_H

#include <asm/registers.h>

#define L1_CACHE_SHIFT 5
/* word-granular cache in microblaze */
#define L1_CACHE_BYTES	(1 << L1_CACHE_SHIFT)

#define SMP_CACHE_BYTES	L1_CACHE_BYTES

#endif /* _ASM_MICROBLAZE_CACHE_H */
