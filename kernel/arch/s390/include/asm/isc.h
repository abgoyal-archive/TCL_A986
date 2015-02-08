
#ifndef _ASM_S390_ISC_H
#define _ASM_S390_ISC_H

#include <linux/types.h>

#define MAX_ISC 7

/* Regular I/O interrupts. */
#define IO_SCH_ISC 3			/* regular I/O subchannels */
#define CONSOLE_ISC 1			/* console I/O subchannel */
#define CHSC_SCH_ISC 7			/* CHSC subchannels */
/* Adapter interrupts. */
#define QDIO_AIRQ_ISC IO_SCH_ISC	/* I/O subchannel in qdio mode */
#define AP_ISC 6			/* adjunct processor (crypto) devices */

/* Functions for registration of I/O interruption subclasses */
void isc_register(unsigned int isc);
void isc_unregister(unsigned int isc);

#endif /* _ASM_S390_ISC_H */
