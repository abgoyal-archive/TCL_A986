


#ifndef MCOMP_X86_H
#define MCOMP_X86_H

#if HAVE_SSE3
#if !CONFIG_RUNTIME_CPU_DETECT

#undef  vp8_search_full_search
#define vp8_search_full_search vp8_full_search_sadx3

#undef  vp8_search_diamond_search
#define vp8_search_diamond_search vp8_diamond_search_sadx4

#endif
#endif

#if HAVE_SSE4_1
#if !CONFIG_RUNTIME_CPU_DETECT

#undef  vp8_search_full_search
#define vp8_search_full_search vp8_full_search_sadx8

#endif
#endif

#endif

