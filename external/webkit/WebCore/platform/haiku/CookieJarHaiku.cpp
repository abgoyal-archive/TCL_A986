

#include "config.h"
#include "CookieJar.h"

#include "Cookie.h"
#include "KURL.h"
#include "PlatformString.h"
#include "StringHash.h"

#include <wtf/HashMap.h>


namespace WebCore {

// FIXME: Shouldn't this be saved to and restored from disk too?
static HashMap<String, String> cookieJar;

void setCookies(Document*, const KURL& url, const String& value)
{
    cookieJar.set(url.string(), value);
}

String cookies(const Document*, const KURL& url)
{
    return cookieJar.get(url.string());
}

String cookieRequestHeaderFieldValue(const Document*, const KURL& url)
{
    // FIXME: include HttpOnly cookies.
    return cookieJar.get(url.string());
}

bool cookiesEnabled(const Document*)
{
    // FIXME: This should probably be a setting
    return true;
}

bool getRawCookies(const Document*, const KURL&, Vector<Cookie>& rawCookies)
{
    // FIXME: Not yet implemented
    rawCookies.clear();
    return false; // return true when implemented
}

void deleteCookie(const Document*, const KURL&, const String&)
{
    // FIXME: Not yet implemented
}

} // namespace WebCore

