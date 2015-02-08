

#include "config.h"

#include "CookieJar.h"

#include "PlatformBridge.h"

namespace WebCore {

void setCookies(Document*, const KURL& url, const String& value)
{
    PlatformBridge::setCookies(url, value);
}

String cookies(const Document*, const KURL& url)
{
    return PlatformBridge::cookies(url);
}

String cookieRequestHeaderFieldValue(const Document*, const KURL& url)
{
    // FIXME: include HttpOnly cookie.
    return PlatformBridge::cookies(url);
}

bool cookiesEnabled(const Document*)
{
    return PlatformBridge::cookiesEnabled();
}

}
