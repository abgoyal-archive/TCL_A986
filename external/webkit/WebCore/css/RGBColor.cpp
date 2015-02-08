

#include "config.h"
#include "RGBColor.h"

namespace WebCore {

PassRefPtr<RGBColor> RGBColor::create(unsigned rgbColor)
{
    return adoptRef(new RGBColor(rgbColor));
}

PassRefPtr<CSSPrimitiveValue> RGBColor::red()
{
    unsigned value = (m_rgbColor >> 16) & 0xFF;
    return CSSPrimitiveValue::create(value, CSSPrimitiveValue::CSS_NUMBER);
}

PassRefPtr<CSSPrimitiveValue> RGBColor::green()
{
    unsigned value = (m_rgbColor >> 8) & 0xFF;
    return CSSPrimitiveValue::create(value, CSSPrimitiveValue::CSS_NUMBER);
}

PassRefPtr<CSSPrimitiveValue> RGBColor::blue()
{
    unsigned value = m_rgbColor & 0xFF;
    return CSSPrimitiveValue::create(value, CSSPrimitiveValue::CSS_NUMBER);
}

PassRefPtr<CSSPrimitiveValue> RGBColor::alpha()
{
    float value = static_cast<float>((m_rgbColor >> 24) & 0xFF) / 0xFF;
    return WebCore::CSSPrimitiveValue::create(value, WebCore::CSSPrimitiveValue::CSS_NUMBER);
}

} // namespace WebCore

