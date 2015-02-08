

#ifndef _SPARC64_DELAY_H
#define _SPARC64_DELAY_H

#ifndef __ASSEMBLY__

extern void __delay(unsigned long loops);
extern void udelay(unsigned long usecs);
#define mdelay(n)	udelay((n) * 1000)

#endif /* !__ASSEMBLY__ */

#endif /* _SPARC64_DELAY_H */
