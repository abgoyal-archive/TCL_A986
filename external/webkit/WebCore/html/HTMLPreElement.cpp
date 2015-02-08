

#include "config.h"
#include "HTMLPreElement.h"

#include "CSSPropertyNames.h"
#include "CSSValueKeywords.h"
#include "HTMLNames.h"
#include "MappedAttribute.h"

namespace WebCore {

using namespace HTMLNames;

HTMLPreElement::HTMLPreElement(const QualifiedName& tagName, Document* doc)
    : HTMLElement(tagName, doc)
{
}

bool HTMLPreElement::mapToEntry(const QualifiedName& attrName, MappedAttributeEntry& result) const
{
    if (attrName == widthAttr || attrName == wrapAttr) {
        result = ePre;
        return false;
    }
    return HTMLElement::mapToEntry(attrName, result);
}

void HTMLPreElement::parseMappedAttribute(MappedAttribute *attr)
{
    if (attr->name() == widthAttr) {
        // FIXME: Implement this some day.  Width on a <pre> is the # of characters that
        // we should size the pre to.  We basically need to take the width of a space,
        // multiply by the value of the attribute and then set that as the width CSS
        // property.
    } else if (attr->name() == wrapAttr) {
        if (!attr->value().isNull())
            addCSSProperty(attr, CSSPropertyWhiteSpace, CSSValuePreWrap);
    } else
        return HTMLElement::parseMappedAttribute(attr);
}

int HTMLPreElement::width() const
{
    return getAttribute(widthAttr).toInt();
}

void HTMLPreElement::setWidth(int width)
{
    setAttribute(widthAttr, String::number(width));
}

bool HTMLPreElement::wrap() const
{
    return !getAttribute(wrapAttr).isNull();
}

void HTMLPreElement::setWrap(bool wrap)
{
    setAttribute(wrapAttr, wrap ? "" : 0);
}

}
