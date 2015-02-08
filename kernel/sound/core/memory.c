

#include <asm/io.h>
#include <asm/uaccess.h>
#include <sound/core.h>

int copy_to_user_fromio(void __user *dst, const volatile void __iomem *src, size_t count)
{
#if defined(__i386__) || defined(CONFIG_SPARC32)
	return copy_to_user(dst, (const void __force*)src, count) ? -EFAULT : 0;
#else
	char buf[256];
	while (count) {
		size_t c = count;
		if (c > sizeof(buf))
			c = sizeof(buf);
		memcpy_fromio(buf, (void __iomem *)src, c);
		if (copy_to_user(dst, buf, c))
			return -EFAULT;
		count -= c;
		dst += c;
		src += c;
	}
	return 0;
#endif
}

EXPORT_SYMBOL(copy_to_user_fromio);

int copy_from_user_toio(volatile void __iomem *dst, const void __user *src, size_t count)
{
#if defined(__i386__) || defined(CONFIG_SPARC32)
	return copy_from_user((void __force *)dst, src, count) ? -EFAULT : 0;
#else
	char buf[256];
	while (count) {
		size_t c = count;
		if (c > sizeof(buf))
			c = sizeof(buf);
		if (copy_from_user(buf, src, c))
			return -EFAULT;
		memcpy_toio(dst, buf, c);
		count -= c;
		dst += c;
		src += c;
	}
	return 0;
#endif
}

EXPORT_SYMBOL(copy_from_user_toio);
