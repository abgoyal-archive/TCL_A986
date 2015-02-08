


/****************************************************************************/
/****************************************************************************/

#ifndef _INTCHW_H
#define _INTCHW_H

/* ---- Include Files ---------------------------------------------------- */
#include <mach/csp/intcHw_reg.h>

/* ---- Public Constants and Types --------------------------------------- */
/* ---- Public Variable Externs ------------------------------------------ */
/* ---- Public Function Prototypes --------------------------------------- */
static inline void intcHw_irq_disable(void *basep, uint32_t mask);
static inline void intcHw_irq_enable(void *basep, uint32_t mask);

#endif /* _INTCHW_H */

