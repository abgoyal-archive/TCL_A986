

#ifndef ResourceRequest_h
#define ResourceRequest_h

#include "ResourceRequestBase.h"

namespace WebCore {

    class Frame;

    class ResourceRequest : public ResourceRequestBase {
    public:
        ResourceRequest(const String& url) 
            : ResourceRequestBase(KURL(ParsedURLString, url), UseProtocolCachePolicy)
            , m_requestorID(0)
            , m_requestorProcessID(0)
            , m_appCacheHostID(0)
        {
        }

        ResourceRequest(const KURL& url) 
            : ResourceRequestBase(url, UseProtocolCachePolicy)
            , m_requestorID(0)
            , m_requestorProcessID(0)
            , m_appCacheHostID(0)
        {
        }

        ResourceRequest(const KURL& url, const String& referrer, ResourceRequestCachePolicy policy = UseProtocolCachePolicy) 
            : ResourceRequestBase(url, policy)
            , m_requestorID(0)
            , m_requestorProcessID(0)
            , m_appCacheHostID(0)
        {
            setHTTPReferrer(referrer);
        }
        
        ResourceRequest()
            : ResourceRequestBase(KURL(), UseProtocolCachePolicy)
            , m_requestorID(0)
            , m_requestorProcessID(0)
            , m_appCacheHostID(0)
        {
        }

        // Allows the request to be matched up with its requestor.
        int requestorID() const { return m_requestorID; }
        void setRequestorID(int requestorID) { m_requestorID = requestorID; }

        // The process id of the process from which this request originated. In
        // the case of out-of-process plugins, this allows to link back the
        // request to the plugin process (as it is processed through a render
        // view process).
        int requestorProcessID() const { return m_requestorProcessID; }
        void setRequestorProcessID(int requestorProcessID) { m_requestorProcessID = requestorProcessID; }

        // Allows the request to be matched up with its app cache host.
        int appCacheHostID() const { return m_appCacheHostID; }
        void setAppCacheHostID(int id) { m_appCacheHostID = id; }

    private:
        friend class ResourceRequestBase;

        void doUpdatePlatformRequest() {}
        void doUpdateResourceRequest() {}

        int m_requestorID;
        int m_requestorProcessID;
        int m_appCacheHostID;
    };

} // namespace WebCore

#endif
