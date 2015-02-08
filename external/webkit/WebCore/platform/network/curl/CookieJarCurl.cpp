

#include "config.h"
#include "CookieJar.h"

#include "Cookie.h"
#include "Document.h"
#include "KURL.h"
#include "PlatformString.h"
#include "StringHash.h"

#include <wtf/HashMap.h>

namespace WebCore {

static HashMap<String, String> cookieJar;

void setCookies(Document* /*document*/, const KURL& url, const String& value)
{
    cookieJar.set(url.string(), value);
}

String cookies(const Document* /*document*/, const KURL& url)
{
    return cookieJar.get(url.string());
}

String cookieRequestHeaderFieldValue(const Document* /*document*/, const KURL& url)
{
    // FIXME: include HttpOnly cookie.
    return cookieJar.get(url.string());
}

bool cookiesEnabled(const Document* /*document*/)
{
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

}
