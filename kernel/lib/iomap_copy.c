

#include <linux/module.h>
#include <linux/io.h>

void __attribute__((weak)) __iowrite32_copy(void __iomem *to,
					    const void *from,
					    size_t count)
{
	u32 __iomem *dst = to;
	const u32 *src = from;
	const u32 *end = src + count;

	while (src < end)
		__raw_writel(*src++, dst++);
}
EXPORT_SYMBOL_GPL(__iowrite32_copy);

void __attribute__((weak)) __iowrite64_copy(void __iomem *to,
					    const void *from,
					    size_t count)
{
#ifdef CONFIG_64BIT
	u64 __iomem *dst = to;
	const u64 *src = from;
	const u64 *end = src + count;

	while (src < end)
		__raw_writeq(*src++, dst++);
#else
	__iowrite32_copy(to, from, count * 2);
#endif
}

EXPORT_SYMBOL_GPL(__iowrite64_copy);
