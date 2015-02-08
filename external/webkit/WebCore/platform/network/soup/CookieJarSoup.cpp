

#include "config.h"
#include "CookieJarSoup.h"

#include "Cookie.h"
#include "CString.h"
#include "Document.h"
#include "GOwnPtrGtk.h"
#include "KURL.h"

namespace WebCore {

static bool cookiesInitialized;
static SoupCookieJar* cookieJar;

SoupCookieJar* defaultCookieJar()
{
    if (!cookiesInitialized) {
        cookiesInitialized = true;
        setDefaultCookieJar(soup_cookie_jar_new());
    }

    return cookieJar;
}

void setDefaultCookieJar(SoupCookieJar* jar)
{
    cookiesInitialized = true;

    if (cookieJar)
        g_object_unref(cookieJar);

    cookieJar = jar;

    if (cookieJar)
        g_object_ref(cookieJar);
}

void setCookies(Document* document, const KURL& url, const String& value)
{
    SoupCookieJar* jar = defaultCookieJar();
    if (!jar)
        return;

    GOwnPtr<SoupURI> origin(soup_uri_new(url.string().utf8().data()));

#ifdef HAVE_LIBSOUP_2_29_90
    GOwnPtr<SoupURI> firstParty(soup_uri_new(document->firstPartyForCookies().string().utf8().data()));

    soup_cookie_jar_set_cookie_with_first_party(jar,
                                                origin.get(),
                                                firstParty.get(),
                                                value.utf8().data());
#else
    soup_cookie_jar_set_cookie(jar,
                               origin.get(),
                               value.utf8().data());
#endif
}

String cookies(const Document* /*document*/, const KURL& url)
{
    SoupCookieJar* jar = defaultCookieJar();
    if (!jar)
        return String();

    SoupURI* uri = soup_uri_new(url.string().utf8().data());
    char* cookies = soup_cookie_jar_get_cookies(jar, uri, FALSE);
    soup_uri_free(uri);

    String result(String::fromUTF8(cookies));
    g_free(cookies);

    return result;
}

String cookieRequestHeaderFieldValue(const Document* /*document*/, const KURL& url)
{
    SoupCookieJar* jar = defaultCookieJar();
    if (!jar)
        return String();

    SoupURI* uri = soup_uri_new(url.string().utf8().data());
    char* cookies = soup_cookie_jar_get_cookies(jar, uri, TRUE);
    soup_uri_free(uri);

    String result(String::fromUTF8(cookies));
    g_free(cookies);

    return result;
}

bool cookiesEnabled(const Document* /*document*/)
{
    return defaultCookieJar();
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
