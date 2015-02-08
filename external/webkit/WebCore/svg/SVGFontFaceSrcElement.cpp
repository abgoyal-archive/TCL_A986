

#include "config.h"

#if ENABLE(SVG_FONTS)
#include "SVGFontFaceSrcElement.h"

#include "CSSValueList.h"
#include "CSSFontFaceSrcValue.h"
#include "SVGFontFaceElement.h"
#include "SVGFontFaceNameElement.h"
#include "SVGFontFaceUriElement.h"
#include "SVGNames.h"

namespace WebCore {
    
using namespace SVGNames;
    
SVGFontFaceSrcElement::SVGFontFaceSrcElement(const QualifiedName& tagName, Document* doc)
    : SVGElement(tagName, doc)
{
}

PassRefPtr<CSSValueList> SVGFontFaceSrcElement::srcValue() const
{
    RefPtr<CSSValueList> list = CSSValueList::createCommaSeparated();
    for (Node* child = firstChild(); child; child = child->nextSibling()) {
        if (child->hasTagName(font_face_uriTag))
            list->append(static_cast<SVGFontFaceUriElement*>(child)->srcValue());
        else if (child->hasTagName(font_face_nameTag))
            list->append(static_cast<SVGFontFaceNameElement*>(child)->srcValue());
    }
    return list;
}

void SVGFontFaceSrcElement::childrenChanged(bool changedByParser, Node* beforeChange, Node* afterChange, int childCountDelta)
{
    SVGElement::childrenChanged(changedByParser, beforeChange, afterChange, childCountDelta);
    if (parentNode() && parentNode()->hasTagName(font_faceTag))
        static_cast<SVGFontFaceElement*>(parentNode())->rebuildFontFace();
}

}

#endif // ENABLE(SVG_FONTS)
