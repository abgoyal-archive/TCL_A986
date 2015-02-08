
 
#ifndef WebKitDefines_h
#define WebKitDefines_h

#ifndef SWIG

#if !wxCHECK_VERSION(2, 9, 0) && wxCHECK_GCC_VERSION(4, 0)
#define WXDLLIMPEXP_WEBKIT __attribute__ ((visibility("default")))
#elif defined(WXMAKINGDLL_WEBKIT)
#define WXDLLIMPEXP_WEBKIT WXEXPORT
#elif defined(WXUSINGDLL_WEBKIT)
#define WXDLLIMPEXP_WEBKIT WXIMPORT
#endif

#else
#define WXDLLIMPEXP_WEBKIT
#endif // SWIG

#endif // WebKitDefines_h
