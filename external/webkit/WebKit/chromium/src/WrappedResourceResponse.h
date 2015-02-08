

#ifndef WrappedResourceResponse_h
#define WrappedResourceResponse_h

// FIXME: This relative path is a temporary hack to support using this
// header from webkit/glue.
#include "../public/WebURLResponse.h"
#include "WebURLResponsePrivate.h"

namespace WebKit {

class WrappedResourceResponse : public WebURLResponse {
public:
    ~WrappedResourceResponse()
    {
        reset(); // Need to drop reference to m_handle
    }

    WrappedResourceResponse() { }

    WrappedResourceResponse(WebCore::ResourceResponse& resourceResponse)
    {
        bind(resourceResponse);
    }

    WrappedResourceResponse(const WebCore::ResourceResponse& resourceResponse)
    {
        bind(resourceResponse);
    }

    void bind(WebCore::ResourceResponse& resourceResponse)
    {
        m_handle.m_resourceResponse = &resourceResponse;
        assign(&m_handle);
    }

    void bind(const WebCore::ResourceResponse& resourceResponse)
    {
        bind(*const_cast<WebCore::ResourceResponse*>(&resourceResponse));
    }

private:
    class Handle : public WebURLResponsePrivate {
    public:
        virtual void dispose() { m_resourceResponse = 0; }
    };

    Handle m_handle;
};

} // namespace WebKit

#endif
