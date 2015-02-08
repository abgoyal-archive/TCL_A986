

#include "config.h"
#include "Language.h"

#include "ChromiumBridge.h"
#include "PlatformString.h"

namespace WebCore {

String defaultLanguage()
{
    static String computedDefaultLanguage;
    if (computedDefaultLanguage.isEmpty())
        computedDefaultLanguage = ChromiumBridge::computedDefaultLanguage();
    return computedDefaultLanguage;
}

} // namespace WebCore
