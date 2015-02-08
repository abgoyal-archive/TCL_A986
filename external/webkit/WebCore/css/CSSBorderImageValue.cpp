

#include "config.h"
#include "CSSBorderImageValue.h"

#include "PlatformString.h"
#include "Rect.h"

namespace WebCore {

CSSBorderImageValue::CSSBorderImageValue(PassRefPtr<CSSValue> image, PassRefPtr<Rect> imageRect, int horizontalRule, int verticalRule)
    : m_image(image)
    , m_imageSliceRect(imageRect)
    , m_horizontalSizeRule(horizontalRule)
    , m_verticalSizeRule(verticalRule)
{
}

String CSSBorderImageValue::cssText() const
{
    // Image first.
    String text(m_image->cssText());
    text += " ";

    // Now the rect, but it isn't really a rect, so we dump manually
    text += m_imageSliceRect->top()->cssText();
    text += " ";
    text += m_imageSliceRect->right()->cssText();
    text += " ";
    text += m_imageSliceRect->bottom()->cssText();
    text += " ";
    text += m_imageSliceRect->left()->cssText();

    // Now the keywords.
    text += " ";
    text += CSSPrimitiveValue::createIdentifier(m_horizontalSizeRule)->cssText();
    text += " ";
    text += CSSPrimitiveValue::createIdentifier(m_verticalSizeRule)->cssText();

    return text;
}

void CSSBorderImageValue::addSubresourceStyleURLs(ListHashSet<KURL>& urls, const CSSStyleSheet* styleSheet)
{
    m_image->addSubresourceStyleURLs(urls, styleSheet);
}

} // namespace WebCore
