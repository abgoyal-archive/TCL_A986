

#include "config.h"
#include "Request.h"

#include "CachedResource.h"

namespace WebCore {

Request::Request(DocLoader* docLoader, CachedResource* object, bool incremental, SecurityCheckPolicy shouldDoSecurityCheck, bool sendResourceLoadCallbacks)
    : m_object(object)
    , m_docLoader(docLoader)
    , m_incremental(incremental)
    , m_multipart(false)
    , m_shouldDoSecurityCheck(shouldDoSecurityCheck)
    , m_sendResourceLoadCallbacks(sendResourceLoadCallbacks)
{
    m_object->setRequest(this);
}

Request::~Request()
{
    m_object->setRequest(0);
}

} //namespace WebCore
