

#include "config.h"
#if ENABLE(SVG)
#include "SVGPaint.h"

namespace WebCore {

SVGPaint::SVGPaint()
    : m_paintType(SVG_PAINTTYPE_UNKNOWN)
{
}

SVGPaint::SVGPaint(const String& uri)
    : m_paintType(SVG_PAINTTYPE_URI_RGBCOLOR)
{
    setUri(uri);
}

SVGPaint::SVGPaint(SVGPaintType paintType)
    : m_paintType(paintType)
{
}

SVGPaint::SVGPaint(SVGPaintType paintType, const String& uri, const String& rgbPaint, const String&)
    : SVGColor(rgbPaint)
    , m_paintType(paintType)
{
    setUri(uri);
}

SVGPaint::SVGPaint(const Color& c)
    : SVGColor(c)
    , m_paintType(SVG_PAINTTYPE_RGBCOLOR)
{
}

SVGPaint::SVGPaint(const String& uri, const Color& c)
    : SVGColor(c)
    , m_paintType(SVG_PAINTTYPE_URI_RGBCOLOR)
{
    setUri(uri);
}

SVGPaint::~SVGPaint()
{
}

SVGPaint* SVGPaint::defaultFill()
{
    static SVGPaint* _defaultFill = new SVGPaint(Color::black);
    return _defaultFill;
}

SVGPaint* SVGPaint::defaultStroke()
{
    static SVGPaint* _defaultStroke = new SVGPaint(SVG_PAINTTYPE_NONE);
    return _defaultStroke;
}

String SVGPaint::uri() const
{
    return m_uri;
}

void SVGPaint::setUri(const String& uri)
{
    m_uri = uri;
}

void SVGPaint::setPaint(SVGPaintType paintType, const String& uri, const String& rgbPaint, const String&, ExceptionCode&)
{
    m_paintType = paintType;

    if (m_paintType == SVG_PAINTTYPE_URI)
        setUri(uri);
    else if (m_paintType == SVG_PAINTTYPE_RGBCOLOR)
        setRGBColor(rgbPaint);
}

String SVGPaint::cssText() const
{
    if (m_paintType == SVG_PAINTTYPE_NONE)
        return "none";
    else if (m_paintType == SVG_PAINTTYPE_CURRENTCOLOR)
        return "currentColor";
    else if (m_paintType == SVG_PAINTTYPE_URI)
        return "url(" + m_uri + ")";

    return SVGColor::cssText();
}

}

// vim:ts=4:noet
#endif // ENABLE(SVG)

