

#ifndef ResourceRequest_h
#define ResourceRequest_h

#include "CachedResource.h"
#include "ResourceRequestBase.h"

namespace WebCore {

class ResourceRequest : public ResourceRequestBase {
public:
    ResourceRequest(const String& url)
        : ResourceRequestBase(KURL(ParsedURLString, url), UseProtocolCachePolicy), m_userGesture(true) { }

    ResourceRequest(const KURL& url) : ResourceRequestBase(url, UseProtocolCachePolicy) , m_userGesture(true) { }

    ResourceRequest(const KURL& url, const String& referrer, ResourceRequestCachePolicy policy = UseProtocolCachePolicy)
        : ResourceRequestBase(url, policy) , m_userGesture(true)
    {
        setHTTPReferrer(referrer);
    }

    ResourceRequest() : ResourceRequestBase(KURL(), UseProtocolCachePolicy), m_userGesture(true) { }

    void doUpdatePlatformRequest() { }
    void doUpdateResourceRequest() { }
    void setUserGesture(bool userGesture) { m_userGesture = userGesture; }
    bool getUserGesture() const { return m_userGesture; }

private:
    friend class ResourceRequestBase;
    bool m_userGesture;
};

} // namespace WebCore

#endif // ResourceRequest_h
