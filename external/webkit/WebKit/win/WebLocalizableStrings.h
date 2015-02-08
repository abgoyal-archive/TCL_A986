

#ifndef WebLocalizableStrings_H
#define WebLocalizableStrings_H

#include <CoreFoundation/CoreFoundation.h>

typedef struct {
    const char *identifier;
    CFBundleRef bundle;
} WebLocalizableStringsBundle;

#ifdef __cplusplus
extern "C" {
#endif

CFStringRef WebLocalizedStringUTF8(WebLocalizableStringsBundle*, LPCSTR key);
LPCTSTR WebLocalizedLPCTSTRUTF8(WebLocalizableStringsBundle*, LPCSTR key);

void SetWebLocalizedStringMainBundle(CFBundleRef bundle);

// These functions are deprecated. Use the UTF-8 versions instead.
CFStringRef WebLocalizedString(WebLocalizableStringsBundle *bundle, LPCTSTR key);
LPCTSTR WebLocalizedLPCTSTR(WebLocalizableStringsBundle *bundle, LPCTSTR key);

#ifdef __cplusplus
}
#endif

#ifdef FRAMEWORK_NAME

#define LOCALIZABLE_STRINGS_BUNDLE(F) LOCALIZABLE_STRINGS_BUNDLE_HELPER(F)
#define LOCALIZABLE_STRINGS_BUNDLE_HELPER(F) F ## LocalizableStringsBundle
extern WebLocalizableStringsBundle LOCALIZABLE_STRINGS_BUNDLE(FRAMEWORK_NAME);

#define UI_STRING(string, comment) WebLocalizedStringUTF8(&LOCALIZABLE_STRINGS_BUNDLE(FRAMEWORK_NAME), string)
#define UI_STRING_KEY(string, key, comment) WebLocalizedStringUTF8(&LOCALIZABLE_STRINGS_BUNDLE(FRAMEWORK_NAME), key)
#define LPCTSTR_UI_STRING(string, comment) WebLocalizedLPCTSTRUTF8(&LOCALIZABLE_STRINGS_BUNDLE(FRAMEWORK_NAME), string)
#define LPCTSTR_UI_STRING_KEY(string, key, comment) WebLocalizedLPCTSTRUTF8(&LOCALIZABLE_STRINGS_BUNDLE(FRAMEWORK_NAME), key)

#else

#define UI_STRING(string, comment) WebLocalizedStringUTF8(0, string)
#define UI_STRING_KEY(string, key, comment) WebLocalizedStringUTF8(0, key)
#define LPCTSTR_UI_STRING(string, comment) WebLocalizedLPCTSTRUTF8(0, string)
#define LPCTSTR_UI_STRING_KEY(string, key, comment) WebLocalizedLPCTSTRUTF8(0, key)

#endif

#endif
