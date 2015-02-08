

#ifndef Cookie_h
#define Cookie_h

#include "PlatformString.h"
#include "StringHash.h"

namespace WebCore {

    // This struct is currently only used to provide more cookies information
    // to the Web Inspector.

    struct Cookie {
        Cookie(const String& name, const String& value, const String& domain,
                const String& path, double expires, bool httpOnly, bool secure,
                bool session)
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

        String name;
        String value;
        String domain;
        String path;
        double expires;
        bool httpOnly;
        bool secure;
        bool session;
    };

    struct CookieHash {
        static unsigned hash(Cookie key)
        { 
            return StringHash::hash(key.name) + StringHash::hash(key.domain) + StringHash::hash(key.path) + key.secure;
        }

        static bool equal(Cookie a, Cookie b)
        {
            return a.name == b.name && a.domain == b.domain && a.path == b.path && a.secure == b.secure;
        }
    };
}

namespace WTF {
    template<typename T> struct DefaultHash;
    template<> struct DefaultHash<WebCore::Cookie> {
        typedef WebCore::CookieHash Hash;
    };
}

#endif
