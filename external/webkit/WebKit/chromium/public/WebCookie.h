

#ifndef WebCookie_h
#define WebCookie_h

#include "WebCommon.h"
#include "WebString.h"

namespace WebKit {

class WebString;

// A cookie.
//
struct WebCookie {
    WebCookie()
        : expires(0)
        , httpOnly(false)
        , secure(false)
        , session(false)
    {
    }

    WebCookie(const WebString& name, const WebString& value, const WebString& domain,
              const WebString& path, double expires, bool httpOnly, bool secure, bool session)
        : name(name)
        , value(value)
        , domain(domain)
        , path(path)
        , expires(expires)
        , httpOnly(httpOnly)
        , secure(secure)
        , session(session)
    {
    }

    WebString name;
    WebString value;
    WebString domain;
    WebString path;
    double expires;
    bool httpOnly;
    bool secure;
    bool session;
};

} // namespace WebKit

#endif
