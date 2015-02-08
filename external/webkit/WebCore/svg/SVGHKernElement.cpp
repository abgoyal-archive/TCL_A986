

#include "config.h"

#if ENABLE(SVG_FONTS)
#include "SVGHKernElement.h"

#include "SVGFontElement.h"
#include "SVGFontFaceElement.h"
#include "SVGFontData.h"
#include "SVGNames.h"
#include "SVGParserUtilities.h"
#include "SimpleFontData.h"
#include "XMLNames.h"

namespace WebCore {

using namespace SVGNames;

SVGHKernElement::SVGHKernElement(const QualifiedName& tagName, Document* doc)
    : SVGElement(tagName, doc)
{
}

SVGHKernElement::~SVGHKernElement()
{
}

void SVGHKernElement::insertedIntoDocument()
{
    Node* fontNode = parentNode();
    if (fontNode && fontNode->hasTagName(SVGNames::fontTag)) {
        if (SVGFontElement* element = static_cast<SVGFontElement*>(fontNode))
            element->invalidateGlyphCache();
    }
}

void SVGHKernElement::removedFromDocument()
{
    Node* fontNode = parentNode();
    if (fontNode && fontNode->hasTagName(SVGNames::fontTag)) {
        if (SVGFontElement* element = static_cast<SVGFontElement*>(fontNode))
            element->invalidateGlyphCache();
    }
}

SVGHorizontalKerningPair SVGHKernElement::buildHorizontalKerningPair() const
{
    SVGHorizontalKerningPair kerningPair;

    kerningPair.unicode1 = getAttribute(u1Attr);
    kerningPair.glyphName1 = getAttribute(g1Attr);
    kerningPair.unicode2 = getAttribute(u2Attr);
    kerningPair.glyphName2 = getAttribute(g2Attr);
    kerningPair.kerning = getAttribute(kAttr).string().toDouble();

    return kerningPair;
}

}

#endif // ENABLE(SVG_FONTS)
