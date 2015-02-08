

#include <linux/kernel.h>

#include <asm/pinmux.h>

int mcf_pinmux_request(unsigned pinmux, unsigned func)
{
	return 0;
}

void mcf_pinmux_release(unsigned pinmux, unsigned func)
{
}
