

#include <linux/module.h>
#include <linux/string.h>

#include <asm/byteorder.h>

static inline unsigned short
from64to16 (unsigned long x)
{
	/* add up 32-bit words for 33 bits */
	x = (x & 0xffffffff) + (x >> 32);
	/* add up 16-bit and 17-bit words for 17+c bits */
	x = (x & 0xffff) + (x >> 16);
	/* add up 16-bit and 2-bit for 16+c bit */
	x = (x & 0xffff) + (x >> 16);
	/* add up carry.. */
	x = (x & 0xffff) + (x >> 16);
	return x;
}

__sum16
csum_tcpudp_magic (__be32 saddr, __be32 daddr, unsigned short len,
		   unsigned short proto, __wsum sum)
{
	return (__force __sum16)~from64to16(
		(__force u64)saddr + (__force u64)daddr +
		(__force u64)sum + ((len + proto) << 8));
}

EXPORT_SYMBOL(csum_tcpudp_magic);

__wsum
csum_tcpudp_nofold (__be32 saddr, __be32 daddr, unsigned short len,
		    unsigned short proto, __wsum sum)
{
	unsigned long result;

	result = (__force u64)saddr + (__force u64)daddr +
		 (__force u64)sum + ((len + proto) << 8);

	/* Fold down to 32-bits so we don't lose in the typedef-less network stack.  */
	/* 64 to 33 */
	result = (result & 0xffffffff) + (result >> 32);
	/* 33 to 32 */
	result = (result & 0xffffffff) + (result >> 32);
	return (__force __wsum)result;
}
EXPORT_SYMBOL(csum_tcpudp_nofold);

extern unsigned long do_csum (const unsigned char *, long);

__wsum csum_partial(const void *buff, int len, __wsum sum)
{
	u64 result = do_csum(buff, len);

	/* add in old sum, and carry.. */
	result += (__force u32)sum;
	/* 32+c bits -> 32 bits */
	result = (result & 0xffffffff) + (result >> 32);
	return (__force __wsum)result;
}

EXPORT_SYMBOL(csum_partial);

__sum16 ip_compute_csum (const void *buff, int len)
{
	return (__force __sum16)~do_csum(buff,len);
}

EXPORT_SYMBOL(ip_compute_csum);
