

#include "config.h"
#include "HTMLUListElement.h"

#include "CSSPropertyNames.h"
#include "HTMLNames.h"
#include "MappedAttribute.h"

namespace WebCore {

using namespace HTMLNames;

HTMLUListElement::HTMLUListElement(const QualifiedName& tagName, Document* doc)
    : HTMLElement(tagName, doc)
{
    ASSERT(hasTagName(ulTag));
}

bool HTMLUListElement::mapToEntry(const QualifiedName& attrName, MappedAttributeEntry& result) const
{
    if (attrName == typeAttr) {
        result = eUnorderedList;
        return false;
    }
    
    return HTMLElement::mapToEntry(attrName, result);
}

void HTMLUListElement::parseMappedAttribute(MappedAttribute *attr)
{
    if (attr->name() == typeAttr)
        addCSSProperty(attr, CSSPropertyListStyleType, attr->value());
    else
        HTMLElement::parseMappedAttribute(attr);
}

bool HTMLUListElement::compact() const
{
    return !getAttribute(compactAttr).isNull();
}

void HTMLUListElement::setCompact(bool b)
{
    setAttribute(compactAttr, b ? "" : 0);
}

String HTMLUListElement::type() const
{
    return getAttribute(typeAttr);
}

void HTMLUListElement::setType(const String &value)
{
    setAttribute(typeAttr, value);
}

}
