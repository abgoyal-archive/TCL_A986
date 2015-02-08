

#include "config.h"
#include "CachedResourceHandle.h"

namespace WebCore {

void CachedResourceHandleBase::setResource(CachedResource* resource) 
{
    if (resource == m_resource)
        return;
    if (m_resource)
        m_resource->unregisterHandle(this);
    m_resource = resource;
    if (m_resource)
        m_resource->registerHandle(this);
}

}
