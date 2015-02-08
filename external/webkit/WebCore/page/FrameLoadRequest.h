

#ifndef FrameLoadRequest_h
#define FrameLoadRequest_h

#include "ResourceRequest.h"

namespace WebCore {

    struct FrameLoadRequest {
    public:
        FrameLoadRequest()
        {
        }

        FrameLoadRequest(const ResourceRequest& resourceRequest)
            : m_resourceRequest(resourceRequest)
        {
        }

        FrameLoadRequest(const ResourceRequest& resourceRequest, const String& frameName)
            : m_resourceRequest(resourceRequest)
            , m_frameName(frameName)
        {
        }

        bool isEmpty() const { return m_resourceRequest.isEmpty(); }

        ResourceRequest& resourceRequest() { return m_resourceRequest; }
        const ResourceRequest& resourceRequest() const { return m_resourceRequest; }

        const String& frameName() const { return m_frameName; }
        void setFrameName(const String& frameName) { m_frameName = frameName; }

    private:
        ResourceRequest m_resourceRequest;
        String m_frameName;
    };

}

#endif // FrameLoadRequest_h

