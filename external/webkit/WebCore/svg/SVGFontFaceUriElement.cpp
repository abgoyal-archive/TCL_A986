

#include "config.h"

#if ENABLE(SVG_FONTS)
#include "SVGFontFaceUriElement.h"

#include "CSSFontFaceSrcValue.h"
#include "CachedFont.h"
#include "DocLoader.h"
#include "Document.h"
#include "MappedAttribute.h"
#include "SVGFontFaceElement.h"
#include "SVGNames.h"
#include "XLinkNames.h"

namespace WebCore {
    
using namespace SVGNames;
    
SVGFontFaceUriElement::SVGFontFaceUriElement(const QualifiedName& tagName, Document* doc)
    : SVGElement(tagName, doc)
{
}

SVGFontFaceUriElement::~SVGFontFaceUriElement()
{
    if (m_cachedFont)
        m_cachedFont->removeClient(this);
}

PassRefPtr<CSSFontFaceSrcValue> SVGFontFaceUriElement::srcValue() const
{
    RefPtr<CSSFontFaceSrcValue> src = CSSFontFaceSrcValue::create(getAttribute(XLinkNames::hrefAttr));
    AtomicString value(getAttribute(formatAttr));
    src->setFormat(value.isEmpty() ? "svg" : value); // Default format
    return src.release();
}

void SVGFontFaceUriElement::parseMappedAttribute(MappedAttribute* attr)
{
    const QualifiedName& attrName = attr->name();
    if (attrName == XLinkNames::hrefAttr)
        loadFont();
    else
        SVGElement::parseMappedAttribute(attr);
}

void SVGFontFaceUriElement::childrenChanged(bool changedByParser, Node* beforeChange, Node* afterChange, int childCountDelta)
{
    SVGElement::childrenChanged(changedByParser, beforeChange, afterChange, childCountDelta);

    if (!parentNode() || !parentNode()->hasTagName(font_face_srcTag))
        return;
    
    Node* grandParent = parentNode()->parentNode();
    if (grandParent && grandParent->hasTagName(font_faceTag))
        static_cast<SVGFontFaceElement*>(grandParent)->rebuildFontFace();
}

void SVGFontFaceUriElement::insertedIntoDocument()
{
    loadFont();
    SVGElement::insertedIntoDocument();
}

void SVGFontFaceUriElement::loadFont()
{
    if (m_cachedFont)
        m_cachedFont->removeClient(this);

    String href = getAttribute(XLinkNames::hrefAttr);
    if (!href.isNull()) {        
        DocLoader* docLoader = document()->docLoader();
        m_cachedFont = docLoader->requestFont(href);
        if (m_cachedFont) {
            m_cachedFont->setSVGFont(true);
            m_cachedFont->addClient(this);
            m_cachedFont->beginLoadIfNeeded(docLoader);
        }
    } else
        m_cachedFont = 0;
}

}

#endif // ENABLE(SVG_FONTS)
