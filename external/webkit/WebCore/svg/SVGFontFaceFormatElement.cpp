

#include "config.h"

#if ENABLE(SVG_FONTS)
#include "SVGFontFaceFormatElement.h"

#include "SVGFontFaceElement.h"
#include "SVGNames.h"

namespace WebCore {
    
using namespace SVGNames;
    
SVGFontFaceFormatElement::SVGFontFaceFormatElement(const QualifiedName& tagName, Document* doc)
    : SVGElement(tagName, doc)
{
}

void SVGFontFaceFormatElement::childrenChanged(bool changedByParser, Node* beforeChange, Node* afterChange, int childCountDelta)
{
    SVGElement::childrenChanged(changedByParser, beforeChange, afterChange, childCountDelta);

    if (!parentNode() || !parentNode()->hasTagName(font_face_uriTag))
        return;
    
    Node* ancestor = parentNode()->parentNode();
    if (!ancestor || !ancestor->hasTagName(font_face_srcTag))
        return;
    
    ancestor = ancestor->parentNode();
    if (ancestor && ancestor->hasTagName(font_faceTag))
        static_cast<SVGFontFaceElement*>(ancestor)->rebuildFontFace();
}

}

#endif // ENABLE(SVG_FONTS)
