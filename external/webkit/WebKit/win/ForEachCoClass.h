

#ifndef ForEachCoClass_h
#define ForEachCoClass_h

#include "ProgIDMacros.h"

#if ENABLE(DATABASE)
#define WEB_DATABASE_MANAGER(macro) macro(WebDatabaseManager)
#else
#define WEB_DATABASE_MANAGER(macro)
#endif

// Items may only be added to the end of this macro. No items may be removed from it.
#define FOR_EACH_COCLASS(macro) \
    macro(CFDictionaryPropertyBag) \
    macro(WebCache) \
    WEB_DATABASE_MANAGER(macro) \
    macro(WebDownload) \
    macro(WebError) \
    macro(WebHistory) \
    macro(WebHistoryItem) \
    macro(WebIconDatabase) \
    macro(WebJavaScriptCollector) \
    macro(WebKitStatistics) \
    macro(WebMutableURLRequest) \
    macro(WebNotificationCenter) \
    macro(WebPreferences) \
    macro(WebScrollBar) \
    macro(WebTextRenderer) \
    macro(WebURLCredential) \
    macro(WebURLProtectionSpace) \
    macro(WebURLRequest) \
    macro(WebURLResponse) \
    macro(WebView) \
    macro(WebArchive) \
    macro(WebCoreStatistics) \
    macro(WebCookieManager) \
    macro(WebWorkersPrivate) \
    macro(WebScriptWorld) \
    macro(WebGeolocationPosition) \
    macro(WebSerializedJSValue) \
    // end of macro

// Everything below this point is deprecated. Please do not use.

#define WEBKITCLASS_MEMBER(cls) cls##Class,
enum WebKitClass {
    FOR_EACH_COCLASS(WEBKITCLASS_MEMBER)
    WebKitClassSentinel
};
#undef WEBKITCLASS_MEMBER

#define OPENSOURCE_PROGID(cls) VERSION_INDEPENDENT_OPENSOURCE_PROGID(cls),
static LPCOLESTR openSourceProgIDs[WebKitClassSentinel] = {
    FOR_EACH_COCLASS(OPENSOURCE_PROGID)
};
#undef OPENSOURCE_PROGID

static LPCOLESTR* s_progIDs = openSourceProgIDs;

#define PROGID(className) progIDForClass(className##Class)

void setUseOpenSourceWebKit(bool);
LPCOLESTR progIDForClass(WebKitClass);

#endif // !defined(ForEachCoClass_h)
