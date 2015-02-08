
 

#include <linux/module.h>
#include <net/checksum.h>

static inline unsigned short from32to16(unsigned long x)
{
	/* add up 16-bit and 16-bit for 16+c bit */
	x = (x & 0xffff) + (x >> 16);
	/* add up carry.. */
	x = (x & 0xffff) + (x >> 16);
	return x;
}

static unsigned long do_csum(const unsigned char * buff, int len)
{
	int odd, count;
	unsigned long result = 0;

	if (len <= 0)
		goto out;
	odd = 1 & (unsigned long) buff;
	if (odd) {
		result = *buff;
		len--;
		buff++;
	}
	count = len >> 1;		/* nr of 16-bit words.. */
	if (count) {
		if (2 & (unsigned long) buff) {
			result += *(unsigned short *) buff;
			count--;
			len -= 2;
			buff += 2;
		}
		count >>= 1;		/* nr of 32-bit words.. */
		if (count) {
		        unsigned long carry = 0;
			do {
				unsigned long w = *(unsigned long *) buff;
				count--;
				buff += 4;
				result += carry;
				result += w;
				carry = (w > result);
			} while (count);
			result += carry;
			result = (result & 0xffff) + (result >> 16);
		}
		if (len & 2) {
			result += *(unsigned short *) buff;
			buff += 2;
		}
	}
	if (len & 1)
		result += (*buff << 8);
	result = from32to16(result);
	if (odd)
		result = ((result >> 8) & 0xff) | ((result & 0xff) << 8);
out:
	return result;
}

#ifdef CONFIG_COLDFIRE
__sum16 ip_fast_csum(const void *iph, unsigned int ihl)
{
	return (__force __sum16)~do_csum(iph,ihl*4);
}
#endif

__wsum csum_partial(const void *buff, int len, __wsum sum)
{
	unsigned int result = do_csum(buff, len);

	/* add in old sum, and carry.. */
	result += (__force u32)sum;
	if ((__force u32)sum > result)
		result += 1;
	return (__force __wsum)result;
}

EXPORT_SYMBOL(csum_partial);


__wsum
csum_partial_copy_from_user(const void __user *src, void *dst,
			    int len, __wsum sum, int *csum_err)
{
	if (csum_err) *csum_err = 0;
	memcpy(dst, (__force const void *)src, len);
	return csum_partial(dst, len, sum);
}


__wsum
csum_partial_copy_nocheck(const void *src, void *dst, int len, __wsum sum)
{
	memcpy(dst, src, len);
	return csum_partial(dst, len, sum);
}
