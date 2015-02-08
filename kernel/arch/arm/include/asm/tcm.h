
#ifndef __ASMARM_TCM_H
#define __ASMARM_TCM_H

//#ifndef CONFIG_HAVE_TCM
//#error "You should not be including tcm.h unless you have a TCM!"
//#endif

#ifdef CONFIG_HAVE_TCM

#include <linux/compiler.h>

/* Tag variables with this */
#define __tcmdata __section(.tcm.data)
/* Tag constants with this */
#define __tcmconst __section(.tcm.rodata)
/* Tag functions inside TCM called from outside TCM with this */
#define __tcmfunc __attribute__((long_call)) __section(.tcm.text) noinline
/* Tag function inside TCM called from inside TCM  with this */
#define __tcmlocalfunc __section(.tcm.text)

void *tcm_alloc(size_t len);
void tcm_free(void *addr, size_t len);

#else

#define __tcmdata
#define __tcmconst 
#define __tcmfunc
#define __tcmlocalfunc

#endif

#endif
