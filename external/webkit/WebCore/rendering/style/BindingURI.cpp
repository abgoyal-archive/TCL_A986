

#include "config.h"
#include "BindingURI.h"

#if ENABLE(XBL)

namespace WebCore {

BindingURI::BindingURI(StringImpl* uri) 
    : m_next(0)
{ 
    m_uri = uri;
    if (uri)
        uri->ref();
}

BindingURI::~BindingURI()
{
    if (m_uri)
        m_uri->deref();
    delete m_next;
}

BindingURI* BindingURI::copy()
{
    BindingURI* newBinding = new BindingURI(m_uri);
    if (next()) {
        BindingURI* nextCopy = next()->copy();
        newBinding->setNext(nextCopy);
    }
    
    return newBinding;
}

bool BindingURI::operator==(const BindingURI& o) const
{
    if ((m_next && !o.m_next) || (!m_next && o.m_next) ||
        (m_next && o.m_next && *m_next != *o.m_next))
        return false;
    
    if (m_uri == o.m_uri)
        return true;
    if (!m_uri || !o.m_uri)
        return false;
    
    return String(m_uri) == String(o.m_uri);
}

} // namespace WebCore

#endif // ENABLE(XBL)
