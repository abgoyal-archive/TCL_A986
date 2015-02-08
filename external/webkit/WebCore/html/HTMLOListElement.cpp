

#include "config.h"
#include "HTMLOListElement.h"

#include "CSSPropertyNames.h"
#include "CSSValueKeywords.h"
#include "HTMLNames.h"
#include "MappedAttribute.h"
#include "RenderListItem.h"

namespace WebCore {

using namespace HTMLNames;

HTMLOListElement::HTMLOListElement(const QualifiedName& tagName, Document* doc)
    : HTMLElement(tagName, doc)
    , m_start(1)
{
    ASSERT(hasTagName(olTag));
}

bool HTMLOListElement::mapToEntry(const QualifiedName& attrName, MappedAttributeEntry& result) const
{
    if (attrName == typeAttr) {
        result = eListItem; // Share with <li>
        return false;
    }
    
    return HTMLElement::mapToEntry(attrName, result);
}

void HTMLOListElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == typeAttr) {
        if (attr->value() == "a")
            addCSSProperty(attr, CSSPropertyListStyleType, CSSValueLowerAlpha);
        else if (attr->value() == "A")
            addCSSProperty(attr, CSSPropertyListStyleType, CSSValueUpperAlpha);
        else if (attr->value() == "i")
            addCSSProperty(attr, CSSPropertyListStyleType, CSSValueLowerRoman);
        else if (attr->value() == "I")
            addCSSProperty(attr, CSSPropertyListStyleType, CSSValueUpperRoman);
        else if (attr->value() == "1")
            addCSSProperty(attr, CSSPropertyListStyleType, CSSValueDecimal);
    } else if (attr->name() == startAttr) {
        bool canParse;
        int start = attr->value().toInt(&canParse);
        if (!canParse)
            start = 1;
        if (start == m_start)
            return;
        m_start = start;
        for (RenderObject* child = renderer(); child; child = child->nextInPreOrder(renderer())) {
            if (child->isListItem())
                toRenderListItem(child)->updateValue();
        }
    } else
        HTMLElement::parseMappedAttribute(attr);
}

bool HTMLOListElement::compact() const
{
    return !getAttribute(compactAttr).isNull();
}

void HTMLOListElement::setCompact(bool b)
{
    setAttribute(compactAttr, b ? "" : 0);
}

void HTMLOListElement::setStart(int start)
{
    setAttribute(startAttr, String::number(start));
}

String HTMLOListElement::type() const
{
    return getAttribute(typeAttr);
}

void HTMLOListElement::setType(const String& value)
{
    setAttribute(typeAttr, value);
}
}
