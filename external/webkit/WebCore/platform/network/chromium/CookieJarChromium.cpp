

#include "config.h"
#include "CookieJar.h"

#include "Cookie.h"
#include "ChromiumBridge.h"
#include "Document.h"

namespace WebCore {

void setCookies(Document* document, const KURL& url, const String& value)
{
    ChromiumBridge::setCookies(url, document->firstPartyForCookies(), value);
}

String cookies(const Document* document, const KURL& url)
{
    return ChromiumBridge::cookies(url, document->firstPartyForCookies());
}

String cookieRequestHeaderFieldValue(const Document* document, const KURL& url)
{
    // FIXME: move in ChromiumBridge?
    Vector<Cookie> cookies;
    getRawCookies(document, url, cookies);
    String cookieLine;
    // FIXME: Set $Version=v;
    for (size_t i = 0; i < cookies.size(); i++) {
        Cookie cookie = cookies[i];
        if (i > 0)
            cookieLine += "; ";
        if (!cookie.name.isEmpty())
            cookieLine += cookie.name + "=";
        cookieLine += cookie.value;
        // FIXME: set $Path, $Domain, ...
    }
    return cookieLine;
}

bool cookiesEnabled(const Document* document)
{
    return ChromiumBridge::cookiesEnabled(document->cookieURL(), document->firstPartyForCookies());
}

bool getRawCookies(const Document* document, const KURL& url, Vector<Cookie>& rawCookies)
{
    return ChromiumBridge::rawCookies(url, document->firstPartyForCookies(), &rawCookies);
}

void deleteCookie(const Document*, const KURL& url, const String& cookieName)
{
    return ChromiumBridge::deleteCookie(url, cookieName);
}

} // namespace WebCore
