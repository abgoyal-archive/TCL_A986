

#ifndef SubstituteResource_h
#define SubstituteResource_h

#include <wtf/RefCounted.h>

#include "KURL.h"
#include "ResourceResponse.h"
#include "SharedBuffer.h"

#include <wtf/RefPtr.h>

namespace WebCore {

class SubstituteResource : public RefCounted<SubstituteResource> {
public:
    virtual ~SubstituteResource() { }

    const KURL& url() const { return m_url; }
    const ResourceResponse& response() const { return m_response; }
    SharedBuffer* data() const { return m_data.get(); }

protected:
    SubstituteResource(const KURL& url, const ResourceResponse& response, PassRefPtr<SharedBuffer> data)
        : m_url(url)
        , m_response(response)
        , m_data(data)
    {
        ASSERT(m_data);
    }
    
private:
    KURL m_url;
    ResourceResponse m_response;
    RefPtr<SharedBuffer> m_data;
};
    
}

#endif // SubstituteResource_h
