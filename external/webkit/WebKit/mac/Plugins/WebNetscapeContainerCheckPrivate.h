

#ifndef WebNetscapeContainerCheckPrivate_h
#define WebNetscapeContainerCheckPrivate_h

#include <WebKit/npapi.h>

#ifdef __cplusplus
extern "C" {
#endif
    
#define WKNVBrowserContainerCheckFuncs 1701
#define WKNVBrowserContainerCheckFuncsVersion 2

#define WKNVBrowserContainerCheckFuncsVersionHasGetLocation 2

typedef uint32 (*WKN_CheckIfAllowedToLoadURLProcPtr)(NPP npp, const char* url, const char* frame, void (*callbackFunc)(NPP npp, uint32, NPBool allowed, void* context), void* context);
typedef void  (*WKN_CancelCheckIfAllowedToLoadURLProcPtr)(NPP npp, uint32);
typedef char* (*WKN_ResolveURLProcPtr)(NPP npp, const char* url, const char* target);

uint32 WKN_CheckIfAllowedToLoadURL(NPP npp, const char* url, const char* frame, void (*callbackFunc)(NPP npp, uint32, NPBool allowed, void* context), void* context);
void WKN_CancelCheckIfAllowedToLoadURL(NPP npp, uint32);
char* WKN_ResolveURL(NPP npp, const char* url, const char* target);

typedef struct _WKNBrowserContainerCheckFuncs {
    uint16 size;
    uint16 version;
    
    WKN_CheckIfAllowedToLoadURLProcPtr checkIfAllowedToLoadURL;
    WKN_CancelCheckIfAllowedToLoadURLProcPtr cancelCheckIfAllowedToLoadURL;
    WKN_ResolveURLProcPtr resolveURL;
} WKNBrowserContainerCheckFuncs;

#ifdef __cplusplus
}
#endif

#endif // WebNetscapeContainerCheckPrivate_h
