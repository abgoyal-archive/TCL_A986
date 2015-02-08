

#include "config.h"
#if ENABLE(SVG)
#include "SVGZoomEvent.h"

namespace WebCore {

SVGZoomEvent::SVGZoomEvent()
    : m_newScale(0.0f)
    , m_previousScale(0.0f)
{
}

SVGZoomEvent::~SVGZoomEvent()
{
}

FloatRect SVGZoomEvent::zoomRectScreen() const
{
    return m_zoomRectScreen;
}

float SVGZoomEvent::previousScale() const
{
    return m_previousScale;
}

void SVGZoomEvent::setPreviousScale(float scale)
{
    m_previousScale = scale;
}

FloatPoint SVGZoomEvent::previousTranslate() const
{
    return m_previousTranslate;
}

float SVGZoomEvent::newScale() const
{
    return m_newScale;
}

void SVGZoomEvent::setNewScale(float scale)
{
    m_newScale = scale;
}

FloatPoint SVGZoomEvent::newTranslate() const
{
    return m_newTranslate;
}

bool SVGZoomEvent::isSVGZoomEvent() const
{
    return true;
}

} // namespace WebCore

#endif // ENABLE(SVG)

// vim:ts=4:noet
