


#ifndef CSP_DELAY_H
#define CSP_DELAY_H

/* ---- Include Files ---------------------------------------------------- */

/* Some CSP routines require use of the following delay routines. Use the OS */
/* version if available, otherwise use a CSP specific definition. */
/* void udelay(unsigned long usecs); */
/* void mdelay(unsigned long msecs); */

#if defined(__KERNEL__) && !defined(STANDALONE)
   #include <linux/delay.h>
#else
   #include <mach/csp/delay.h>
#endif

/* ---- Public Constants and Types --------------------------------------- */
/* ---- Public Variable Externs ------------------------------------------ */
/* ---- Public Function Prototypes --------------------------------------- */

#endif /*  CSP_DELAY_H */
