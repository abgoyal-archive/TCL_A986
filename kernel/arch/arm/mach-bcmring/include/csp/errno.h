

#ifndef CSP_ERRNO_H
#define CSP_ERRNO_H

/* ---- Include Files ---------------------------------------------------- */

#if   defined(__KERNEL__)
#include <linux/errno.h>
#elif defined(CSP_SIMULATION)
#include <asm-generic/errno.h>
#else
#include <errno.h>
#endif

/* ---- Public Constants and Types --------------------------------------- */
/* ---- Public Variable Externs ------------------------------------------ */
/* ---- Public Function Prototypes --------------------------------------- */

#endif /* CSP_ERRNO_H */
