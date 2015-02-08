

#include "config.h"
#if ENABLE(SVG)
#include "SVGColor.h"

#include "CSSParser.h"
#include "RGBColor.h"
#include "SVGException.h"

namespace WebCore {

SVGColor::SVGColor()
    : m_colorType(SVG_COLORTYPE_UNKNOWN)
{
}

SVGColor::SVGColor(const String& rgbColor)
    : m_colorType(SVG_COLORTYPE_RGBCOLOR)
{
    setRGBColor(rgbColor);
}

SVGColor::SVGColor(SVGColorType colorType)
    : m_colorType(colorType)
{
}

SVGColor::SVGColor(const Color& c)
    : m_color(c)
    , m_colorType(SVG_COLORTYPE_RGBCOLOR)
{
}


SVGColor::~SVGColor()
{
}

unsigned short SVGColor::colorType() const
{
    return m_colorType;
}

PassRefPtr<RGBColor> SVGColor::rgbColor() const
{
    return RGBColor::create(m_color.rgb());
}

void SVGColor::setRGBColor(const String& rgbColor, ExceptionCode& ec)
{
    Color color = SVGColor::colorFromRGBColorString(rgbColor);
    if (color.isValid())
        m_color = color;
    else
        ec = SVGException::SVG_INVALID_VALUE_ERR;
}

Color SVGColor::colorFromRGBColorString(const String& colorString)
{
    String s = colorString.stripWhiteSpace();
    // hsl, hsla and rgba are not in the SVG spec.
    // FIXME: rework css parser so it is more svg aware
    if (s.startsWith("hsl") || s.startsWith("rgba"))
        return Color();
    RGBA32 color;
    if (CSSParser::parseColor(color, s))
        return color;
    return Color();
}

void SVGColor::setRGBColorICCColor(const String& /* rgbColor */, const String& /* iccColor */, ExceptionCode&)
{
    // TODO: implement me!
}

void SVGColor::setColor(unsigned short colorType, const String& /* rgbColor */ , const String& /* iccColor */, ExceptionCode&)
{
    // TODO: implement me!
    m_colorType = colorType;
}

String SVGColor::cssText() const
{
    if (m_colorType == SVG_COLORTYPE_RGBCOLOR)
        return m_color.name();

    return String();
}

const Color& SVGColor::color() const
{
    return m_color;
}

}

// vim:ts=4:noet
#endif // ENABLE(SVG)

