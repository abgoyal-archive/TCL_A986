


#include <linux/highmem.h>
#include <linux/vmalloc.h>
#include "drmP.h"


#if __OS_HAS_AGP

#ifdef HAVE_PAGE_AGP
#include <asm/agp.h>
#else
# ifdef __powerpc__
#  define PAGE_AGP	__pgprot(_PAGE_KERNEL | _PAGE_NO_CACHE)
# else
#  define PAGE_AGP	PAGE_KERNEL
# endif
#endif

#else				/* __OS_HAS_AGP */

#endif
