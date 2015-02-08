

#include "config.h"
#include "Language.h"

#include "CString.h"
#include "PlatformString.h"

namespace WebCore {

static String localeInfo(LCTYPE localeType, const String& fallback)
{
    LANGID langID = GetUserDefaultUILanguage();
    int localeChars = GetLocaleInfo(langID, localeType, 0, 0);
    if (!localeChars)
        return fallback;
    UChar* localeNameBuf;
    String localeName = String::createUninitialized(localeChars, localeNameBuf);
    localeChars = GetLocaleInfo(langID, localeType, localeNameBuf, localeChars);
    if (!localeChars)
        return fallback;
    if (localeName.isEmpty())
        return fallback;

    return localeName;
}

String defaultLanguage()
{
    static String computedDefaultLanguage;
    if (!computedDefaultLanguage.isEmpty())
        return computedDefaultLanguage;

    String languageName = localeInfo(LOCALE_SISO639LANGNAME, "en");
    String countryName = localeInfo(LOCALE_SISO3166CTRYNAME, String());

    if (countryName.isEmpty())
        computedDefaultLanguage = languageName;
    else
        computedDefaultLanguage = String::format("%s-%s", languageName.latin1().data(), countryName.latin1().data());

    return computedDefaultLanguage;
}

}
