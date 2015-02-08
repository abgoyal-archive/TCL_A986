

#include "config.h"
#include "HTMLBRElement.h"

#include "CSSPropertyNames.h"
#include "HTMLNames.h"
#include "MappedAttribute.h"
#include "RenderBR.h"

namespace WebCore {

using namespace HTMLNames;

HTMLBRElement::HTMLBRElement(const QualifiedName& tagName, Document *doc)
    : HTMLElement(tagName, doc)
{
    ASSERT(hasTagName(brTag));
}

bool HTMLBRElement::mapToEntry(const QualifiedName& attrName, MappedAttributeEntry& result) const
{
    if (attrName == clearAttr) {
        result = eUniversal;
        return false;
    }
    
    return HTMLElement::mapToEntry(attrName, result);
}

void HTMLBRElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == clearAttr) {
        // If the string is empty, then don't add the clear property. 
        // <br clear> and <br clear=""> are just treated like <br> by Gecko, Mac IE, etc. -dwh
        const AtomicString& str = attr->value();
        if (!str.isEmpty()) {
            if (equalIgnoringCase(str, "all"))
                addCSSProperty(attr, CSSPropertyClear, "both");
            else
                addCSSProperty(attr, CSSPropertyClear, str);
        }
    } else
        HTMLElement::parseMappedAttribute(attr);
}

RenderObject* HTMLBRElement::createRenderer(RenderArena* arena, RenderStyle* style)
{
     if (style->contentData())
        return RenderObject::createObject(this, style);

     return new (arena) RenderBR(this);
}

}
