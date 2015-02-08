

#ifndef CSP_CACHE_H
#define CSP_CACHE_H

/* ---- Include Files ---------------------------------------------------- */

#include <csp/stdint.h>

/* ---- Public Constants and Types --------------------------------------- */

#if defined(__KERNEL__) && !defined(STANDALONE)
#include <asm/cacheflush.h>

#define CSP_CACHE_FLUSH_ALL      flush_cache_all()

#else

#define CSP_CACHE_FLUSH_ALL

#endif

#endif /* CSP_CACHE_H */
