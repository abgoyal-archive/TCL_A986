

#include "config.h"
#include "HTMLDivElement.h"

#include "CSSPropertyNames.h"
#include "CSSValueKeywords.h"
#include "HTMLNames.h"
#include "MappedAttribute.h"

namespace WebCore {

using namespace HTMLNames;

HTMLDivElement::HTMLDivElement(const QualifiedName& tagName, Document *doc)
    : HTMLElement(tagName, doc)
{
    ASSERT(hasTagName(divTag));
}

HTMLDivElement::~HTMLDivElement()
{
}

bool HTMLDivElement::mapToEntry(const QualifiedName& attrName, MappedAttributeEntry& result) const
{
    if (attrName == alignAttr) {
        result = eBlock;
        return false;
    }
    return HTMLElement::mapToEntry(attrName, result);
}

void HTMLDivElement::parseMappedAttribute(MappedAttribute *attr)
{
    if (attr->name() == alignAttr) {
        String v = attr->value();
        if (equalIgnoringCase(attr->value(), "middle") || equalIgnoringCase(attr->value(), "center"))
           addCSSProperty(attr, CSSPropertyTextAlign, CSSValueWebkitCenter);
        else if (equalIgnoringCase(attr->value(), "left"))
            addCSSProperty(attr, CSSPropertyTextAlign, CSSValueWebkitLeft);
        else if (equalIgnoringCase(attr->value(), "right"))
            addCSSProperty(attr, CSSPropertyTextAlign, CSSValueWebkitRight);
        else
            addCSSProperty(attr, CSSPropertyTextAlign, v);
    } else
        HTMLElement::parseMappedAttribute(attr);
}

String HTMLDivElement::align() const
{
    return getAttribute(alignAttr);
}

void HTMLDivElement::setAlign(const String &value)
{
    setAttribute(alignAttr, value);
}

}
