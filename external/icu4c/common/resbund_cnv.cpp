
#include "unicode/utypes.h"
#include "unicode/resbund.h"
#include "uinvchar.h"

U_NAMESPACE_BEGIN

ResourceBundle::ResourceBundle( const UnicodeString&    path,
                                const Locale&           locale,
                                UErrorCode&              error)
                                :UObject(), fLocale(NULL)
{
    constructForLocale(path, locale, error);
}

ResourceBundle::ResourceBundle( const UnicodeString&    path,
                                UErrorCode&              error)
                                :UObject(), fLocale(NULL)
{
    constructForLocale(path, Locale::getDefault(), error);
}

void 
ResourceBundle::constructForLocale(const UnicodeString& path,
                                   const Locale& locale,
                                   UErrorCode& error)
{
    if (path.isEmpty()) {
        fResource = ures_open(NULL, locale.getName(), &error);
    }
    else {
        UnicodeString nullTerminatedPath(path);
        nullTerminatedPath.append((UChar)0);
        fResource = ures_openU(nullTerminatedPath.getBuffer(), locale.getName(), &error);
    }
}

U_NAMESPACE_END
