

#include "config.h"

#if ENABLE(SVG_FONTS)
#include "SVGFontFaceNameElement.h"

#include "CSSFontFaceSrcValue.h"
#include "SVGNames.h"

namespace WebCore {
    
SVGFontFaceNameElement::SVGFontFaceNameElement(const QualifiedName& tagName, Document* doc)
    : SVGElement(tagName, doc)
{
}

PassRefPtr<CSSFontFaceSrcValue> SVGFontFaceNameElement::srcValue() const
{
    return CSSFontFaceSrcValue::createLocal(getAttribute(SVGNames::nameAttr));
}

}

#endif // ENABLE(SVG)

