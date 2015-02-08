

#ifndef ResourceRequest_h
#define ResourceRequest_h

#include "ResourceRequestBase.h"

#include <wtf/RetainPtr.h>
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
        
        ResourceRequest(CFURLRequestRef cfRequest)
            : ResourceRequestBase()
            , m_cfRequest(cfRequest) { }
        
        CFURLRequestRef cfURLRequest() const;       

    private:
        friend class ResourceRequestBase;

        void doUpdatePlatformRequest();
        void doUpdateResourceRequest();
        
        RetainPtr<CFURLRequestRef> m_cfRequest;      
    };

} // namespace WebCore

#endif // ResourceRequest_h
