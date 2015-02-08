

#include "config.h"
#include "CookieJar.h"

#include "Cookie.h"
#include "KURL.h"
#include "PlatformString.h"
#include "Document.h"
#include "ResourceHandle.h"
#include <windows.h>
#include <Wininet.h>

namespace WebCore {

void setCookies(Document* /*document*/, const KURL& url, const String& value)
{
    // FIXME: Deal with document->firstPartyForCookies().
    String str = url.string();
    String val = value;
    InternetSetCookie(str.charactersWithNullTermination(), 0, val.charactersWithNullTermination());
}

String cookies(const Document* /*document*/, const KURL& url)
{
    String str = url.string();

    DWORD count = str.length() + 1;
    InternetGetCookie(str.charactersWithNullTermination(), 0, 0, &count);
    if (count <= 1) // Null terminator counts as 1.
        return String();

    Vector<UChar> buffer(count);
    InternetGetCookie(str.charactersWithNullTermination(), 0, buffer.data(), &count);
    buffer.shrink(count - 1); // Ignore the null terminator.
    return String::adopt(buffer);
}

String cookieRequestHeaderFieldValue(const Document* document, const KURL& url)
{
    // FIXME: include HttpOnly cookie
    return cookies(document, url);
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
