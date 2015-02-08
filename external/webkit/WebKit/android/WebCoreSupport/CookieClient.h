

#ifndef COOKIE_CLIENT_H
#define COOKIE_CLIENT_H

#include <KURL.h>
#include <PlatformString.h>

using namespace WebCore;

namespace android {

class CookieClient {

public:
    virtual ~CookieClient() {}
    virtual void setCookies(const KURL& url, const String& value) = 0;
    virtual String cookies(const KURL& url) = 0;
    virtual bool cookiesEnabled() = 0;
};

}
#endif
