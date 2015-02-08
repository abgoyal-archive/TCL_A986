

#include "config.h"
#include "HTMLLIElement.h"

#include "CSSPropertyNames.h"
#include "CSSValueKeywords.h"
#include "HTMLNames.h"
#include "MappedAttribute.h"
#include "RenderListItem.h"

namespace WebCore {

using namespace HTMLNames;

HTMLLIElement::HTMLLIElement(const QualifiedName& tagName, Document* doc)
    : HTMLElement(tagName, doc)
    , m_requestedValue(0)
{
    ASSERT(hasTagName(liTag));
}

bool HTMLLIElement::mapToEntry(const QualifiedName& attrName, MappedAttributeEntry& result) const
{
    if (attrName == typeAttr) {
        result = eListItem; // Share with <ol> since all the values are the same
        return false;
    }
    
    return HTMLElement::mapToEntry(attrName, result);
}

void HTMLLIElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == valueAttr) {
        m_requestedValue = attr->value().toInt();
        if (renderer() && renderer()->isListItem()) {
            if (m_requestedValue > 0)
                toRenderListItem(renderer())->setExplicitValue(m_requestedValue);
            else
                toRenderListItem(renderer())->clearExplicitValue();
        }
    } else if (attr->name() == typeAttr) {
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
        else
            addCSSProperty(attr, CSSPropertyListStyleType, attr->value());
    } else
        HTMLElement::parseMappedAttribute(attr);
}

void HTMLLIElement::attach()
{
    ASSERT(!attached());

    HTMLElement::attach();

    if (renderer() && renderer()->isListItem()) {
        RenderListItem* render = toRenderListItem(renderer());

        // Find the enclosing list node.
        Node* listNode = 0;
        Node* n = this;
        while (!listNode && (n = n->parentNode())) {
            if (n->hasTagName(ulTag) || n->hasTagName(olTag))
                listNode = n;
        }

        // If we are not in a list, tell the renderer so it can position us inside.
        // We don't want to change our style to say "inside" since that would affect nested nodes.
        if (!listNode)
            render->setNotInList(true);

        if (m_requestedValue > 0)
            render->setExplicitValue(m_requestedValue);
        else
            render->clearExplicitValue();
    }
}

String HTMLLIElement::type() const
{
    return getAttribute(typeAttr);
}

void HTMLLIElement::setType(const String& value)
{
    setAttribute(typeAttr, value);
}

int HTMLLIElement::value() const
{
    return getAttribute(valueAttr).toInt();
}

void HTMLLIElement::setValue(int value)
{
    setAttribute(valueAttr, String::number(value));
}

}
