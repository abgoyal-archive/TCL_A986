

#ifndef WrappedResourceRequest_h
#define WrappedResourceRequest_h

// FIXME: This relative path is a temporary hack to support using this
// header from webkit/glue.
#include "../public/WebURLRequest.h"
#include "WebURLRequestPrivate.h"

namespace WebKit {

class WrappedResourceRequest : public WebURLRequest {
public:
    ~WrappedResourceRequest()
    {
        reset(); // Need to drop reference to m_handle
    }

    WrappedResourceRequest() { }

    WrappedResourceRequest(WebCore::ResourceRequest& resourceRequest)
    {
        bind(resourceRequest);
    }

    WrappedResourceRequest(const WebCore::ResourceRequest& resourceRequest)
    {
        bind(resourceRequest);
    }

    void bind(WebCore::ResourceRequest& resourceRequest)
    {
        m_handle.m_resourceRequest = &resourceRequest;
        assign(&m_handle);
    }

    void bind(const WebCore::ResourceRequest& resourceRequest)
    {
        bind(*const_cast<WebCore::ResourceRequest*>(&resourceRequest));
    }

private:
    class Handle : public WebURLRequestPrivate {
    public:
        virtual void dispose() { m_resourceRequest = 0; }
    };

    Handle m_handle;
};

} // namespace WebKit

#endif
