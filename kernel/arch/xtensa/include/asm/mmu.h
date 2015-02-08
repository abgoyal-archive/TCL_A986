

#ifndef _XTENSA_MMU_H
#define _XTENSA_MMU_H

#ifndef CONFIG_MMU
#include <asm/nommu.h>
#else

/* Default "unsigned long" context */
typedef unsigned long mm_context_t;

#endif /* CONFIG_MMU */
#endif	/* _XTENSA_MMU_H */
