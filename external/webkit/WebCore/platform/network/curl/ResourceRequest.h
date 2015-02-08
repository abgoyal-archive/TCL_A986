

#ifndef ResourceRequest_h
#define ResourceRequest_h

#include "ResourceRequestBase.h"

typedef const struct _CFURLRequest* CFURLRequestRef;

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

        // Needed for compatibility.
        CFURLRequestRef cfURLRequest() const { return 0; }

    private:
        friend class ResourceRequestBase;

        void doUpdatePlatformRequest() {}
        void doUpdateResourceRequest() {}
    };

} // namespace WebCore

#endif // ResourceRequest_h
