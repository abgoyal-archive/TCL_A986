

#include "config.h"
#include "HTMLParagraphElement.h"

#include "CSSPropertyNames.h"
#include "CSSValueKeywords.h"
#include "Document.h"
#include "HTMLNames.h"
#include "MappedAttribute.h"

namespace WebCore {

using namespace HTMLNames;

HTMLParagraphElement::HTMLParagraphElement(const QualifiedName& tagName, Document *doc)
    : HTMLElement(tagName, doc)
{
    ASSERT(hasTagName(pTag));
}

bool HTMLParagraphElement::checkDTD(const Node* newChild)
{
    return inInlineTagList(newChild) || (document()->inCompatMode() && newChild->hasTagName(tableTag));
}

bool HTMLParagraphElement::mapToEntry(const QualifiedName& attrName, MappedAttributeEntry& result) const
{
    if (attrName == alignAttr) {
        result = eBlock; // We can share with DIV here.
        return false;
    }
    return HTMLElement::mapToEntry(attrName, result);
}

void HTMLParagraphElement::parseMappedAttribute(MappedAttribute *attr)
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

String HTMLParagraphElement::align() const
{
    return getAttribute(alignAttr);
}

void HTMLParagraphElement::setAlign(const String &value)
{
    setAttribute(alignAttr, value);
}

}
