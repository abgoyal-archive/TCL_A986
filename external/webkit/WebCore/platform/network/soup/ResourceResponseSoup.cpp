

#include "config.h"
#include "ResourceResponse.h"

#include "CString.h"
#include "GOwnPtr.h"
#include "PlatformString.h"

using namespace std;

namespace WebCore {

SoupMessage* ResourceResponse::toSoupMessage() const
{
    // This GET here is just because SoupMessage wants it, we dn't really know.
    SoupMessage* soupMessage = soup_message_new("GET", url().string().utf8().data());
    if (!soupMessage)
        return 0;

    soupMessage->status_code = httpStatusCode();

    HTTPHeaderMap headers = httpHeaderFields();
    SoupMessageHeaders* soupHeaders = soupMessage->response_headers;
    if (!headers.isEmpty()) {
        HTTPHeaderMap::const_iterator end = headers.end();
        for (HTTPHeaderMap::const_iterator it = headers.begin(); it != end; ++it)
            soup_message_headers_append(soupHeaders, it->first.string().utf8().data(), it->second.utf8().data());
    }

    // Body data is not in the message.
    return soupMessage;
}

void ResourceResponse::updateFromSoupMessage(SoupMessage* soupMessage)
{
    SoupURI* soupURI = soup_message_get_uri(soupMessage);
    GOwnPtr<gchar> uri(soup_uri_to_string(soupURI, FALSE));
    m_url = KURL(KURL(), String::fromUTF8(uri.get()));

    m_httpStatusCode = soupMessage->status_code;

    SoupMessageHeadersIter headersIter;
    const char* headerName;
    const char* headerValue;

    soup_message_headers_iter_init(&headersIter, soupMessage->response_headers);
    while (soup_message_headers_iter_next(&headersIter, &headerName, &headerValue))
        m_httpHeaderFields.set(String::fromUTF8(headerName), String::fromUTF8(headerValue));
}

}
