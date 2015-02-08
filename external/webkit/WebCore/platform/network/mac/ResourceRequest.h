

#ifndef ResourceRequest_h
#define ResourceRequest_h

#include "ResourceRequestBase.h"

#include <wtf/RetainPtr.h>
#ifdef __OBJC__
@class NSURLRequest;
#else
class NSURLRequest;
#endif

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
        
        ResourceRequest(NSURLRequest* nsRequest)
            : ResourceRequestBase()
            , m_nsRequest(nsRequest) { }
        
        void applyWebArchiveHackForMail();
        NSURLRequest* nsURLRequest() const;

    private:
        friend class ResourceRequestBase;

        void doUpdatePlatformRequest();
        void doUpdateResourceRequest();
        
        RetainPtr<NSURLRequest> m_nsRequest;
    };

} // namespace WebCore

#endif // ResourceRequest_h
