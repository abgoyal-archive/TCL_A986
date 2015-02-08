#ifndef _LINUX_BITOPS_H
#define _LINUX_BITOPS_H
#include <asm/types.h>

#include <asm/bitops.h>

static __inline__ int get_bitmask_order(unsigned int count)
{
	int order;
	
	order = fls(count);
	return order;	/* We could be slightly more clever with -1 here... */
}

static __inline__ int get_count_order(unsigned int count)
{
	int order;
	
	order = fls(count) - 1;
	if (count & (count - 1))
		order++;
	return order;
}

static inline unsigned long hweight_long(unsigned long w)
{
	return sizeof(w) == 4 ? hweight32(w) : hweight64(w);
}

static inline __u32 rol32(__u32 word, unsigned int shift)
{
	return (word << shift) | (word >> (32 - shift));
}

static inline __u32 ror32(__u32 word, unsigned int shift)
{
	return (word >> shift) | (word << (32 - shift));
}

static inline unsigned fls_long(unsigned long l)
{
	if (sizeof(l) == 4)
		return fls(l);
	return fls64(l);
}

#endif
