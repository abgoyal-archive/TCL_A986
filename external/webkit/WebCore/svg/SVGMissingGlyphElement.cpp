

#include "config.h"

#if ENABLE(SVG_FONTS)
#include "SVGMissingGlyphElement.h"

namespace WebCore {

SVGMissingGlyphElement::SVGMissingGlyphElement(const QualifiedName& tagName, Document* doc)
    : SVGStyledElement(tagName, doc)
{
}

}

#endif // ENABLE(SVG_FONTS)
