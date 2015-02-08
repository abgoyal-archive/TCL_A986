

#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/hvcall.h>
#include <asm/hvconsole.h>
#include "plpar_wrappers.h"

int hvc_get_chars(uint32_t vtermno, char *buf, int count)
{
	unsigned long got;

	if (plpar_get_term_char(vtermno, &got, buf) == H_SUCCESS)
		return got;

	return 0;
}

EXPORT_SYMBOL(hvc_get_chars);


int hvc_put_chars(uint32_t vtermno, const char *buf, int count)
{
	unsigned long *lbuf = (unsigned long *) buf;
	long ret;


	/* hcall will ret H_PARAMETER if 'count' exceeds firmware max.*/
	if (count > MAX_VIO_PUT_CHARS)
		count = MAX_VIO_PUT_CHARS;

	ret = plpar_hcall_norets(H_PUT_TERM_CHAR, vtermno, count, lbuf[0],
				 lbuf[1]);
	if (ret == H_SUCCESS)
		return count;
	if (ret == H_BUSY)
		return 0;
	return -EIO;
}

EXPORT_SYMBOL(hvc_put_chars);
