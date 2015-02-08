

#ifndef _AU1XXX_H_
#define _AU1XXX_H_

#include <asm/mach-au1x00/au1000.h>

#if defined(CONFIG_MIPS_DB1000) || defined(CONFIG_MIPS_DB1100) || \
    defined(CONFIG_MIPS_DB1500) || defined(CONFIG_MIPS_DB1550)
#include <asm/mach-db1x00/db1x00.h>

#elif defined(CONFIG_MIPS_PB1550)
#include <asm/mach-pb1x00/pb1550.h>

#elif defined(CONFIG_MIPS_PB1200)
#include <asm/mach-pb1x00/pb1200.h>

#elif defined(CONFIG_MIPS_DB1200)
#include <asm/mach-db1x00/db1200.h>

#endif

#endif /* _AU1XXX_H_ */
