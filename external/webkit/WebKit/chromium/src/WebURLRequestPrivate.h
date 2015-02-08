

#ifndef WebURLRequestPrivate_h
#define WebURLRequestPrivate_h

namespace WebCore { class ResourceRequest; }

namespace WebKit {

class WebURLRequestPrivate {
public:
    WebURLRequestPrivate() : m_resourceRequest(0), m_allowStoredCredentials(true) { }

    // Called by WebURLRequest when it no longer needs this object.
    virtual void dispose() = 0;

    WebCore::ResourceRequest* m_resourceRequest;
    bool m_allowStoredCredentials;
};

} // namespace WebKit

#endif
