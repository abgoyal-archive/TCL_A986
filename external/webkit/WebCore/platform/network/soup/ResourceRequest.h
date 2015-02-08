

#ifndef ResourceRequest_h
#define ResourceRequest_h

#include "ResourceRequestBase.h"

#include <libsoup/soup.h>

namespace WebCore {

    class ResourceRequest : public ResourceRequestBase {
    public:
        ResourceRequest(const String& url)
            : ResourceRequestBase(KURL(ParsedURLString, url), UseProtocolCachePolicy)
        {
        }

        ResourceRequest(const KURL& url)
            : ResourceRequestBase(url, UseProtocolCachePolicy)
        {
        }

        ResourceRequest(const KURL& url, const String& referrer, ResourceRequestCachePolicy policy = UseProtocolCachePolicy)
            : ResourceRequestBase(url, policy)
        {
            setHTTPReferrer(referrer);
        }

        ResourceRequest()
            : ResourceRequestBase(KURL(), UseProtocolCachePolicy)
        {
        }

        ResourceRequest(SoupMessage* soupMessage)
            : ResourceRequestBase(KURL(), UseProtocolCachePolicy)
        {
            updateFromSoupMessage(soupMessage);
        }

        SoupMessage* toSoupMessage() const;
        void updateFromSoupMessage(SoupMessage* soupMessage);

    private:
        friend class ResourceRequestBase;

        void doUpdatePlatformRequest() {};
        void doUpdateResourceRequest() {};
    };

} // namespace WebCore

#endif // ResourceRequest_h
