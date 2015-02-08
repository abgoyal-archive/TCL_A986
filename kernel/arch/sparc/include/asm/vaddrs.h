
#ifndef _SPARC_VADDRS_H
#define _SPARC_VADDRS_H

#include <asm/head.h>


#define SRMMU_MAXMEM		0x0c000000

#define SRMMU_NOCACHE_VADDR	(KERNBASE + SRMMU_MAXMEM)
				/* = 0x0fc000000 */
/* XXX Empiricals - this needs to go away - KMW */
#define SRMMU_MIN_NOCACHE_PAGES (550)
#define SRMMU_MAX_NOCACHE_PAGES	(1280)

#define SRMMU_NOCACHE_ALCRATIO	64	/* 256 pages per 64MB of system RAM */

#define SUN4M_IOBASE_VADDR	0xfd000000 /* Base for mapping pages */
#define IOBASE_VADDR		0xfe000000
#define IOBASE_END		0xfe600000

#define SUN4C_LOCK_VADDR	0xff000000
#define SUN4C_LOCK_END		0xffc00000

#define KADB_DEBUGGER_BEGVM	0xffc00000 /* Where kern debugger is in virt-mem */
#define KADB_DEBUGGER_ENDVM	0xffd00000
#define DEBUG_FIRSTVADDR	KADB_DEBUGGER_BEGVM
#define DEBUG_LASTVADDR		KADB_DEBUGGER_ENDVM

#define LINUX_OPPROM_BEGVM	0xffd00000
#define LINUX_OPPROM_ENDVM	0xfff00000

#define DVMA_VADDR		0xfff00000 /* Base area of the DVMA on suns */
#define DVMA_END		0xfffc0000

#endif /* !(_SPARC_VADDRS_H) */
