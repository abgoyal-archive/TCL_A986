

#ifndef CookieJarSoup_h
#define CookieJarSoup_h

#include "CookieJar.h"
#include <libsoup/soup.h>

namespace WebCore {
    SoupCookieJar* defaultCookieJar();
    void setDefaultCookieJar(SoupCookieJar* jar);
}

#endif
