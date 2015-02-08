

#include "config.h"
#include "TextBreakIteratorInternalICU.h"

#include "CString.h"
#include "Language.h"
#include "PlatformString.h"
#include <wtf/StdLibExtras.h>

namespace WebCore {

static const char* UILanguage()
{
    // Chrome's UI language can be different from the OS UI language on Windows.
    // We want to return Chrome's UI language here.
    DEFINE_STATIC_LOCAL(CString, locale, (defaultLanguage().latin1()));
    return locale.data();
}

const char* currentSearchLocaleID()
{
    return UILanguage();
}

const char* currentTextBreakLocaleID()
{
    return UILanguage();
}

} // namespace WebCore
