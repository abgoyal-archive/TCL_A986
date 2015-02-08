

#ifndef DumpRenderTreeMac_h
#define DumpRenderTreeMac_h

// FIXME: we should add a config.h file for DumpRenderTree.
#define WTF_PLATFORM_CF 1

#if !defined(MAC_OS_X_VERSION_10_5) || MAC_OS_X_VERSION_MAX_ALLOWED < MAC_OS_X_VERSION_10_5
#define BUILDING_ON_TIGER 1
#elif !defined(MAC_OS_X_VERSION_10_6) || MAC_OS_X_VERSION_MAX_ALLOWED < MAC_OS_X_VERSION_10_6
#define BUILDING_ON_LEOPARD 1
#elif !defined(MAC_OS_X_VERSION_10_7) || MAC_OS_X_VERSION_MAX_ALLOWED < MAC_OS_X_VERSION_10_7
#define BUILDING_ON_SNOW_LEOPARD 1
#endif

@class DumpRenderTreeDraggingInfo;
@class NavigationController;
@class PolicyDelegate;
@class WebFrame;
@class WebScriptWorld;
@class WebView;

typedef const struct __CFString* CFStringRef;

extern CFMutableArrayRef openWindowsRef;
extern CFMutableSetRef disallowedURLs;
extern WebFrame* mainFrame;
extern WebFrame* topLoadingFrame;
extern DumpRenderTreeDraggingInfo *draggingInfo;
extern NavigationController* gNavigationController;
extern PolicyDelegate* policyDelegate;

extern const unsigned maxViewHeight;
extern const unsigned maxViewWidth;

extern CFRunLoopTimerRef waitToDumpWatchdog;

WebView* createWebViewAndOffscreenWindow();
void setPersistentUserStyleSheetLocation(CFStringRef);

unsigned worldIDForWorld(WebScriptWorld *);

#endif // DumpRenderTreeMac_h 
