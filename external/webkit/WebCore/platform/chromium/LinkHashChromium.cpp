

#include "config.h"
#include "LinkHash.h"

#include "ChromiumBridge.h"

namespace WebCore {

LinkHash visitedLinkHash(const UChar* url, unsigned length)
{
    return ChromiumBridge::visitedLinkHash(url, length);
}

LinkHash visitedLinkHash(const KURL& base, const AtomicString& attributeURL)
{
    return ChromiumBridge::visitedLinkHash(base, attributeURL);
}

} // namespace WebCore
