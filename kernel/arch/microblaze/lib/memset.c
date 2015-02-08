

#include <linux/types.h>
#include <linux/stddef.h>
#include <linux/compiler.h>
#include <linux/module.h>
#include <linux/string.h>

#ifdef __HAVE_ARCH_MEMSET
void *memset(void *v_src, int c, __kernel_size_t n)
{
	char *src = v_src;
#ifdef CONFIG_OPT_LIB_FUNCTION
	uint32_t *i_src;
	uint32_t w32 = 0;
#endif
	/* Truncate c to 8 bits */
	c = (c & 0xFF);

#ifdef CONFIG_OPT_LIB_FUNCTION
	if (unlikely(c)) {
		/* Make a repeating word out of it */
		w32 = c;
		w32 |= w32 << 8;
		w32 |= w32 << 16;
	}

	if (likely(n >= 4)) {
		/* Align the destination to a word boundary */
		/* This is done in an endian independant manner */
		switch ((unsigned) src & 3) {
		case 1:
			*src++ = c;
			--n;
		case 2:
			*src++ = c;
			--n;
		case 3:
			*src++ = c;
			--n;
		}

		i_src  = (void *)src;

		/* Do as many full-word copies as we can */
		for (; n >= 4; n -= 4)
			*i_src++ = w32;

		src  = (void *)i_src;
	}
#endif
	/* Simple, byte oriented memset or the rest of count. */
	while (n--)
		*src++ = c;

	return v_src;
}
EXPORT_SYMBOL(memset);
#endif /* __HAVE_ARCH_MEMSET */
