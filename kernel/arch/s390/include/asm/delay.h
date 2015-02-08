
 
#ifndef _S390_DELAY_H
#define _S390_DELAY_H

extern void __udelay(unsigned long long usecs);
extern void udelay_simple(unsigned long long usecs);
extern void __delay(unsigned long loops);

#define udelay(n) __udelay((unsigned long long) (n))
#define mdelay(n) __udelay((unsigned long long) (n) * 1000)

#endif /* defined(_S390_DELAY_H) */
