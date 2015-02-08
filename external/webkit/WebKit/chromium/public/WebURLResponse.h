

#ifndef WebURLResponse_h
#define WebURLResponse_h

#include "WebCommon.h"

#if defined(WEBKIT_IMPLEMENTATION)
namespace WebCore { class ResourceResponse; }
#endif

namespace WebKit {

class WebCString;
class WebHTTPHeaderVisitor;
class WebString;
class WebURL;
class WebURLResponsePrivate;

class WebURLResponse {
public:
    ~WebURLResponse() { reset(); }

    WebURLResponse() : m_private(0) { }
    WebURLResponse(const WebURLResponse& r) : m_private(0) { assign(r); }
    WebURLResponse& operator=(const WebURLResponse& r)
    {
        assign(r);
        return *this;
    }

    explicit WebURLResponse(const WebURL& url) : m_private(0)
    {
        initialize();
        setURL(url);
    }

    WEBKIT_API void initialize();
    WEBKIT_API void reset();
    WEBKIT_API void assign(const WebURLResponse&);

    WEBKIT_API bool isNull() const;

    WEBKIT_API WebURL url() const;
    WEBKIT_API void setURL(const WebURL&);

    WEBKIT_API WebString mimeType() const;
    WEBKIT_API void setMIMEType(const WebString&);

    WEBKIT_API long long expectedContentLength() const;
    WEBKIT_API void setExpectedContentLength(long long);

    WEBKIT_API WebString textEncodingName() const;
    WEBKIT_API void setTextEncodingName(const WebString&);

    WEBKIT_API WebString suggestedFileName() const;
    WEBKIT_API void setSuggestedFileName(const WebString&);

    WEBKIT_API int httpStatusCode() const;
    WEBKIT_API void setHTTPStatusCode(int);

    WEBKIT_API WebString httpStatusText() const;
    WEBKIT_API void setHTTPStatusText(const WebString&);

    WEBKIT_API WebString httpHeaderField(const WebString& name) const;
    WEBKIT_API void setHTTPHeaderField(const WebString& name, const WebString& value);
    WEBKIT_API void addHTTPHeaderField(const WebString& name, const WebString& value);
    WEBKIT_API void clearHTTPHeaderField(const WebString& name);
    WEBKIT_API void visitHTTPHeaderFields(WebHTTPHeaderVisitor*) const;

    WEBKIT_API double lastModifiedDate() const;
    WEBKIT_API void setLastModifiedDate(double);

    WEBKIT_API bool isContentFiltered() const;
    WEBKIT_API void setIsContentFiltered(bool);

    WEBKIT_API long long appCacheID() const;
    WEBKIT_API void setAppCacheID(long long);

    WEBKIT_API WebURL appCacheManifestURL() const;
    WEBKIT_API void setAppCacheManifestURL(const WebURL&);

    // A consumer controlled value intended to be used to record opaque
    // security info related to this request.
    WEBKIT_API WebCString securityInfo() const;
    WEBKIT_API void setSecurityInfo(const WebCString&);

#if defined(WEBKIT_IMPLEMENTATION)
    WebCore::ResourceResponse& toMutableResourceResponse();
    const WebCore::ResourceResponse& toResourceResponse() const;
#endif

    // Flag whether this request was loaded via the SPDY protocol or not.
    // SPDY is an experimental web protocol, see http://dev.chromium.org/spdy
    WEBKIT_API bool wasFetchedViaSPDY() const;
    WEBKIT_API void setWasFetchedViaSPDY(bool);

protected:
    void assign(WebURLResponsePrivate*);

private:
    WebURLResponsePrivate* m_private;
};

} // namespace WebKit

#endif
