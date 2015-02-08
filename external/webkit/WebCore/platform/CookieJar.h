

#ifndef CookieJar_h
#define CookieJar_h

#include <wtf/Vector.h>

namespace WebCore {

    class Document;
    class KURL;
    class String;

    struct Cookie;

    // cookies omits HttpOnly cookies.
    String cookies(const Document*, const KURL&);
    String cookieRequestHeaderFieldValue(const Document*, const KURL&);
    void setCookies(Document*, const KURL&, const String&);
    bool cookiesEnabled(const Document*);
    bool getRawCookies(const Document*, const KURL&, Vector<Cookie>&);
    void deleteCookie(const Document*, const KURL&, const String&);

}

#endif
