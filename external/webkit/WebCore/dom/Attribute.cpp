

#include "config.h"
#include "Attribute.h"

#include "Attr.h"
#include "Element.h"

namespace WebCore {

PassRefPtr<Attribute> Attribute::clone() const
{
    return adoptRef(new Attribute(m_name, m_value));
}

PassRefPtr<Attr> Attribute::createAttrIfNeeded(Element* e)
{
    RefPtr<Attr> r = m_impl;
    if (!r)
        r = Attr::create(e, e->document(), this);
    return r.release();
}

}
