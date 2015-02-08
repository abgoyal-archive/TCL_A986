

#include <linux/module.h>
#include <linux/types.h>
#include <linux/string.h>

#include <net/checksum.h>
#include <asm/byteorder.h>
#include <asm/uaccess.h>

__wsum
csum_partial_copy_nocheck (const void *src, void *dst, int len, __wsum sum)
{
	sum = csum_partial(src, len, sum);
	memcpy(dst, src, len);

	return sum;
}
EXPORT_SYMBOL(csum_partial_copy_nocheck);

__wsum
csum_partial_copy_from_user (const void __user *src, void *dst,
			     int len, __wsum sum, int *err_ptr)
{
	int missing;

	missing = copy_from_user(dst, src, len);
	if (missing) {
		memset(dst + len - missing, 0, missing);
		*err_ptr = -EFAULT;
	}

	return csum_partial(dst, len-missing, sum);
}
EXPORT_SYMBOL(csum_partial_copy_from_user);
EXPORT_SYMBOL(csum_partial);
