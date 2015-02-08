

#include "config.h"
#include "NamedMappedAttrMap.h"

#include "Document.h"
#include "Element.h"
#include "MappedAttribute.h"

namespace WebCore {

void NamedMappedAttrMap::clearAttributes()
{
    m_classNames.clear();
    m_mappedAttributeCount = 0;
    NamedNodeMap::clearAttributes();
}

bool NamedMappedAttrMap::isMappedAttributeMap() const
{
    return true;
}

int NamedMappedAttrMap::declCount() const
{
    int result = 0;
    for (unsigned i = 0; i < length(); i++) {
        Attribute* attr = attributeItem(i);
        if (attr->isMappedAttribute() &&
            static_cast<MappedAttribute*>(attr)->decl())
            result++;
    }
    return result;
}

bool NamedMappedAttrMap::mapsEquivalent(const NamedMappedAttrMap* otherMap) const
{
    // The # of decls must match.
    if (declCount() != otherMap->declCount())
        return false;
    
    // The values for each decl must match.
    for (unsigned i = 0; i < length(); i++) {
        Attribute* attr = attributeItem(i);
        if (attr->isMappedAttribute() &&
            static_cast<MappedAttribute*>(attr)->decl()) {
            Attribute* otherAttr = otherMap->getAttributeItem(attr->name());
            if (!otherAttr || (attr->value() != otherAttr->value()))
                return false;
        }
    }
    return true;
}

void NamedMappedAttrMap::setClass(const String& classStr) 
{ 
    if (!element()->hasClass()) { 
        m_classNames.clear(); 
        return;
    }

    m_classNames.set(classStr, element()->document()->inCompatMode()); 
}

}
