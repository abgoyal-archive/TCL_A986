

#include "config.h"

#if ENABLE(WML)
#include "WMLBRElement.h"

#include "CSSPropertyNames.h"
#include "HTMLNames.h"
#include "MappedAttribute.h"
#include "RenderBR.h"

namespace WebCore {

WMLBRElement::WMLBRElement(const QualifiedName& tagName, Document* doc)
    : WMLElement(tagName, doc)
{
}

bool WMLBRElement::mapToEntry(const QualifiedName& attrName, MappedAttributeEntry& result) const
{
    if (attrName == HTMLNames::clearAttr) {
        result = eUniversal;
        return false;
    }

    return WMLElement::mapToEntry(attrName, result);
}

void WMLBRElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == HTMLNames::clearAttr) {
        // If the string is empty, then don't add the clear property. 
        // <br clear> and <br clear=""> are just treated like <br> by Gecko, Mac IE, etc. -dwh
        const AtomicString& value = attr->value();
        if (value.isEmpty())
            return;

        if (equalIgnoringCase(value, "all"))
            addCSSProperty(attr, CSSPropertyClear, "both");
        else
            addCSSProperty(attr, CSSPropertyClear, value);
    } else
        WMLElement::parseMappedAttribute(attr);
}

RenderObject* WMLBRElement::createRenderer(RenderArena* arena, RenderStyle*)
{
    return new (arena) RenderBR(this);
}

}

#endif
