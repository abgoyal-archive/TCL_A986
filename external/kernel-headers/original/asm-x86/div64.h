#ifndef _ASM_X86_DIV64_H
#define _ASM_X86_DIV64_H

#ifdef CONFIG_X86_32

#include <linux/types.h>

#define do_div(n,base) ({ \
	unsigned long __upper, __low, __high, __mod, __base; \
	__base = (base); \
	asm("":"=a" (__low), "=d" (__high):"A" (n)); \
	__upper = __high; \
	if (__high) { \
		__upper = __high % (__base); \
		__high = __high / (__base); \
	} \
	asm("divl %2":"=a" (__low), "=d" (__mod):"rm" (__base), "0" (__low), "1" (__upper)); \
	asm("":"=A" (n):"a" (__low),"d" (__high)); \
	__mod; \
})

#define div_long_long_rem(a,b,c) div_ll_X_l_rem(a,b,c)

static inline long
div_ll_X_l_rem(long long divs, long div, long *rem)
{
	long dum2;
      __asm__("divl %2":"=a"(dum2), "=d"(*rem)
      :	"rm"(div), "A"(divs));

	return dum2;

}

extern uint64_t div64_64(uint64_t dividend, uint64_t divisor);

#else
# include <asm-generic/div64.h>
#endif /* CONFIG_X86_32 */

#endif /* _ASM_X86_DIV64_H */
