

#include "config.h"
#include "ResourceRequest.h"

#include "CString.h"
#include "GOwnPtr.h"
#include "GOwnPtrGtk.h"
#include "PlatformString.h"

#include <libsoup/soup.h>

using namespace std;

namespace WebCore {

SoupMessage* ResourceRequest::toSoupMessage() const
{
    SoupMessage* soupMessage = soup_message_new(httpMethod().utf8().data(), url().string().utf8().data());
    if (!soupMessage)
        return 0;

    HTTPHeaderMap headers = httpHeaderFields();
    SoupMessageHeaders* soupHeaders = soupMessage->request_headers;
    if (!headers.isEmpty()) {
        HTTPHeaderMap::const_iterator end = headers.end();
        for (HTTPHeaderMap::const_iterator it = headers.begin(); it != end; ++it)
            soup_message_headers_append(soupHeaders, it->first.string().utf8().data(), it->second.utf8().data());
    }

#ifdef HAVE_LIBSOUP_2_29_90
    String firstPartyString = firstPartyForCookies().string();
    if (!firstPartyString.isEmpty()) {
        GOwnPtr<SoupURI> firstParty(soup_uri_new(firstPartyString.utf8().data()));
        soup_message_set_first_party(soupMessage, firstParty.get());
    }
#endif

    // Body data is only handled at ResourceHandleSoup::startHttp for
    // now; this is because this may not be a good place to go
    // openning and mmapping files. We should maybe revisit this.
    return soupMessage;
}

void ResourceRequest::updateFromSoupMessage(SoupMessage* soupMessage)
{
    SoupURI* soupURI = soup_message_get_uri(soupMessage);
    GOwnPtr<gchar> uri(soup_uri_to_string(soupURI, FALSE));
    m_url = KURL(KURL(), String::fromUTF8(uri.get()));

    m_httpMethod = String::fromUTF8(soupMessage->method);

    SoupMessageHeadersIter headersIter;
    const char* headerName;
    const char* headerValue;

    soup_message_headers_iter_init(&headersIter, soupMessage->request_headers);
    while (soup_message_headers_iter_next(&headersIter, &headerName, &headerValue))
        m_httpHeaderFields.set(String::fromUTF8(headerName), String::fromUTF8(headerValue));

    if (soupMessage->request_body->data)
        m_httpBody = FormData::create(soupMessage->request_body->data, soupMessage->request_body->length);

#ifdef HAVE_LIBSOUP_2_29_90
    SoupURI* firstParty = soup_message_get_first_party(soupMessage);
    if (firstParty) {
        GOwnPtr<gchar> firstPartyURI(soup_uri_to_string(firstParty, FALSE));
        m_firstPartyForCookies = KURL(KURL(), String::fromUTF8(firstPartyURI.get()));
    }
#endif

    // FIXME: m_allowCookies should probably be handled here and on
    // doUpdatePlatformRequest somehow.
}

unsigned initializeMaximumHTTPConnectionCountPerHost()
{
    // Soup has its own queue control; it wants to have all requests
    // given to it, so that it is able to look ahead, and schedule
    // them in a good way.
    return 10000;
}

}
