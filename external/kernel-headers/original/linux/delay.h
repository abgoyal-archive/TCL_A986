#ifndef _LINUX_DELAY_H
#define _LINUX_DELAY_H


extern unsigned long loops_per_jiffy;

#include <asm/delay.h>


#ifndef MAX_UDELAY_MS
#define MAX_UDELAY_MS	5
#endif

#ifndef mdelay
#define mdelay(n) (\
	(__builtin_constant_p(n) && (n)<=MAX_UDELAY_MS) ? udelay((n)*1000) : \
	({unsigned long __ms=(n); while (__ms--) udelay(1000);}))
#endif

#ifndef ndelay
#define ndelay(x)	udelay(((x)+999)/1000)
#endif

void calibrate_delay(void);
void msleep(unsigned int msecs);
unsigned long msleep_interruptible(unsigned int msecs);

static inline void ssleep(unsigned int seconds)
{
	msleep(seconds * 1000);
}

#endif /* defined(_LINUX_DELAY_H) */
