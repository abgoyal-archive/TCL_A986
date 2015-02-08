

#include "config.h"
#include "HTMLTablePartElement.h"

#include "CSSHelper.h"
#include "CSSPropertyNames.h"
#include "CSSValueKeywords.h"
#include "Document.h"
#include "HTMLNames.h"
#include "MappedAttribute.h"

namespace WebCore {

using namespace HTMLNames;

bool HTMLTablePartElement::mapToEntry(const QualifiedName& attrName, MappedAttributeEntry& result) const
{
    if (attrName == backgroundAttr) {
        result = (MappedAttributeEntry)(eLastEntry + document()->docID());
        return false;
    }
    
    if (attrName == bgcolorAttr ||
        attrName == bordercolorAttr ||
        attrName == valignAttr ||
        attrName == heightAttr) {
        result = eUniversal;
        return false;
    }
    
    if (attrName == alignAttr) {
        result = eCell; // All table parts will just share in the TD space.
        return false;
    }

    return HTMLElement::mapToEntry(attrName, result);
}

void HTMLTablePartElement::parseMappedAttribute(MappedAttribute *attr)
{
    if (attr->name() == bgcolorAttr)
        addCSSColor(attr, CSSPropertyBackgroundColor, attr->value());
    else if (attr->name() == backgroundAttr) {
        String url = deprecatedParseURL(attr->value());
        if (!url.isEmpty())
            addCSSImageProperty(attr, CSSPropertyBackgroundImage, document()->completeURL(url).string());
    } else if (attr->name() == bordercolorAttr) {
        if (!attr->value().isEmpty()) {
            addCSSColor(attr, CSSPropertyBorderColor, attr->value());
            addCSSProperty(attr, CSSPropertyBorderTopStyle, CSSValueSolid);
            addCSSProperty(attr, CSSPropertyBorderBottomStyle, CSSValueSolid);
            addCSSProperty(attr, CSSPropertyBorderLeftStyle, CSSValueSolid);
            addCSSProperty(attr, CSSPropertyBorderRightStyle, CSSValueSolid);
        }
    } else if (attr->name() == valignAttr) {
        if (!attr->value().isEmpty())
            addCSSProperty(attr, CSSPropertyVerticalAlign, attr->value());
    } else if (attr->name() == alignAttr) {
        const AtomicString& v = attr->value();
        if (equalIgnoringCase(v, "middle") || equalIgnoringCase(v, "center"))
            addCSSProperty(attr, CSSPropertyTextAlign, CSSValueWebkitCenter);
        else if (equalIgnoringCase(v, "absmiddle"))
            addCSSProperty(attr, CSSPropertyTextAlign, CSSValueCenter);
        else if (equalIgnoringCase(v, "left"))
            addCSSProperty(attr, CSSPropertyTextAlign, CSSValueWebkitLeft);
        else if (equalIgnoringCase(v, "right"))
            addCSSProperty(attr, CSSPropertyTextAlign, CSSValueWebkitRight);
        else
            addCSSProperty(attr, CSSPropertyTextAlign, v);
    } else if (attr->name() == heightAttr) {
        if (!attr->value().isEmpty())
            addCSSLength(attr, CSSPropertyHeight, attr->value());
    } else
        HTMLElement::parseMappedAttribute(attr);
}

}
