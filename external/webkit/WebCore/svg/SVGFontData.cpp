

#include "config.h"

#if ENABLE(SVG_FONTS)
#include "SVGFontData.h"

namespace WebCore {

SVGFontData::SVGFontData(SVGFontFaceElement* fontFaceElement)
    : m_svgFontFaceElement(fontFaceElement)
    , m_horizontalOriginX(fontFaceElement->horizontalOriginX())
    , m_horizontalOriginY(fontFaceElement->horizontalOriginY())
    , m_horizontalAdvanceX(fontFaceElement->horizontalAdvanceX())
    , m_verticalOriginX(fontFaceElement->verticalOriginX())
    , m_verticalOriginY(fontFaceElement->verticalOriginY())
    , m_verticalAdvanceY(fontFaceElement->verticalAdvanceY())
{
}

SVGFontData::~SVGFontData()
{
}

} // namespace WebCore

#endif
