

#ifndef __HIWebView__
#define __HIWebView__

#ifndef __LP64__

#include <Carbon/Carbon.h>

#include <JavaScriptCore/WebKitAvailability.h>

#if PRAGMA_ONCE
#pragma once
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __OBJC__
@class WebView;
#endif

extern OSStatus 
HIWebViewCreate(HIViewRef * outControl) AVAILABLE_WEBKIT_VERSION_1_0_AND_LATER_BUT_DEPRECATED_IN_WEBKIT_VERSION_4_0;

#ifdef __OBJC__

extern WebView *
HIWebViewGetWebView(HIViewRef inView) AVAILABLE_WEBKIT_VERSION_1_0_AND_LATER_BUT_DEPRECATED_IN_WEBKIT_VERSION_4_0;

#endif

#ifdef __cplusplus
}
#endif

#endif
#endif /* __HIWebView__ */
