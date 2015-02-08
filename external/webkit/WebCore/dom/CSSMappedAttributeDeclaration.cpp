
#include "config.h"
#include "CSSMappedAttributeDeclaration.h"

#include "StyledElement.h"

namespace WebCore {

CSSMappedAttributeDeclaration::~CSSMappedAttributeDeclaration()
{
    if (m_entryType != ePersistent)
        StyledElement::removeMappedAttributeDecl(m_entryType, m_attrName, m_attrValue);
}

}
