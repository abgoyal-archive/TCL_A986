

#include "config.h"
#include "HTMLTableCaptionElement.h"

#include "CSSPropertyNames.h"
#include "HTMLNames.h"
#include "MappedAttribute.h"

namespace WebCore {

using namespace HTMLNames;

HTMLTableCaptionElement::HTMLTableCaptionElement(const QualifiedName& tagName, Document *doc)
    : HTMLTablePartElement(tagName, doc)
{
    ASSERT(hasTagName(captionTag));
}

bool HTMLTableCaptionElement::mapToEntry(const QualifiedName& attrName, MappedAttributeEntry& result) const
{
    if (attrName == alignAttr) {
        result = eCaption;
        return false;
    }

    return HTMLElement::mapToEntry(attrName, result);
}

void HTMLTableCaptionElement::parseMappedAttribute(MappedAttribute *attr)
{
    if (attr->name() == alignAttr) {
        if (!attr->value().isEmpty())
            addCSSProperty(attr, CSSPropertyCaptionSide, attr->value());
    } else
        HTMLElement::parseMappedAttribute(attr);
}

String HTMLTableCaptionElement::align() const
{
    return getAttribute(alignAttr);
}

void HTMLTableCaptionElement::setAlign(const String &value)
{
    setAttribute(alignAttr, value);
}

}
