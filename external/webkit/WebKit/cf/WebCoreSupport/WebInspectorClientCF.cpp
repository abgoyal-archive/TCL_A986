

// FIXME: On Windows, we require all WebKit source files to include config.h
// before including any other files. Failing to include config.h will leave
// WTF_PLATFORM_CF and WTF_USE_JSC undefined, causing build failures in this 
// file. But Mac doesn't have a config.h for WebKit, so we can't include the 
// Windows one here. For now we can just define WTF_PLATFORM_CF and WTF_USE_JSC
// manually, but we need a better long-term solution.
#ifndef WTF_PLATFORM_CF
#define WTF_PLATFORM_CF 1
#endif

#ifndef WTF_USE_JSC
#define WTF_USE_JSC 1
#endif

#if defined(WIN32) || defined(_WIN32)
#if defined(BUILDING_JavaScriptCore) || defined(BUILDING_WTF)
#define JS_EXPORTDATA __declspec(dllexport)
#else
#define JS_EXPORTDATA __declspec(dllimport)
#endif
#define JS_EXPORTCLASS JS_EXPORTDATA
#else
#define JS_EXPORTDATA
#define JS_EXPORTCLASS
#endif

#include "WebInspectorClient.h"

#include <CoreFoundation/CoreFoundation.h>

#include <WebCore/PlatformString.h>

#include <wtf/RetainPtr.h>
#include <wtf/Vector.h>

using namespace WebCore;

static inline CFStringRef createKeyForPreferences(const String& key)
{
    RetainPtr<CFStringRef> keyCFString(AdoptCF, key.createCFString());
    return CFStringCreateWithFormat(0, 0, CFSTR("WebKit Web Inspector Setting - %@"), keyCFString.get());
}

void WebInspectorClient::populateSetting(const String& key, String* setting)
{
    RetainPtr<CFStringRef> preferencesKey(AdoptCF, createKeyForPreferences(key));
    RetainPtr<CFPropertyListRef> value(AdoptCF, CFPreferencesCopyAppValue(preferencesKey.get(), kCFPreferencesCurrentApplication));

    if (!value)
        return;

    CFTypeID type = CFGetTypeID(value.get());
    if (type == CFStringGetTypeID())
        *setting = static_cast<String>(static_cast<CFStringRef>(value.get()));
    else if (type == CFBooleanGetTypeID())
        *setting = static_cast<bool>(CFBooleanGetValue(static_cast<CFBooleanRef>(value.get()))) ? "true" : "false";
    else
        *setting = "";
}

void WebInspectorClient::storeSetting(const String& key, const String& setting)
{
    RetainPtr<CFPropertyListRef> objectToStore;
    objectToStore.adoptCF(setting.createCFString());
    ASSERT(objectToStore);

    RetainPtr<CFStringRef> preferencesKey(AdoptCF, createKeyForPreferences(key));
    CFPreferencesSetAppValue(preferencesKey.get(), objectToStore.get(), kCFPreferencesCurrentApplication);
}
