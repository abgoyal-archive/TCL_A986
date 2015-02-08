

#include "config.h"
#include "MappedAttribute.h"

namespace WebCore {

PassRefPtr<Attribute> MappedAttribute::clone() const
{
    return adoptRef(new MappedAttribute(name(), value(), m_styleDecl.get()));
}

}
