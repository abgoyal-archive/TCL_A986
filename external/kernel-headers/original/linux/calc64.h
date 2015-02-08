#ifndef _LINUX_CALC64_H
#define _LINUX_CALC64_H

#include <linux/types.h>
#include <asm/div64.h>

#ifndef div_long_long_rem
#define div_long_long_rem(dividend, divisor, remainder)	\
	do_div_llr((dividend), divisor, remainder)

static inline unsigned long do_div_llr(const long long dividend,
				       const long divisor, long *remainder)
{
	u64 result = dividend;

	*(remainder) = do_div(result, divisor);
	return (unsigned long) result;
}
#endif

static inline long div_long_long_rem_signed(const long long dividend,
					    const long divisor, long *remainder)
{
	long res;

	if (unlikely(dividend < 0)) {
		res = -div_long_long_rem(-dividend, divisor, remainder);
		*remainder = -(*remainder);
	} else
		res = div_long_long_rem(dividend, divisor, remainder);

	return res;
}

#endif
