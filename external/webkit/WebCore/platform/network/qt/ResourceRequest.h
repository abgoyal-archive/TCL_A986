

#ifndef ResourceRequest_h
#define ResourceRequest_h

#include "ResourceRequestBase.h"

QT_BEGIN_NAMESPACE
class QNetworkRequest;
class QObject;
QT_END_NAMESPACE

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

        QNetworkRequest toNetworkRequest(QObject* originatingObject) const;

    private:
        friend class ResourceRequestBase;

        void doUpdatePlatformRequest() {}
        void doUpdateResourceRequest() {}
    };

} // namespace WebCore

#endif // ResourceRequest_h
