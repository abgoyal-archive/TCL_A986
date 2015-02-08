

#ifndef RGBColor_h
#define RGBColor_h

#include "CSSPrimitiveValue.h"
#include "Color.h"
#include <wtf/RefCounted.h>

namespace WebCore {

    class RGBColor : public RefCounted<RGBColor> {
    public:
        static PassRefPtr<RGBColor> create(unsigned rgbColor);

        PassRefPtr<CSSPrimitiveValue> red();
        PassRefPtr<CSSPrimitiveValue> green();
        PassRefPtr<CSSPrimitiveValue> blue();
        PassRefPtr<CSSPrimitiveValue> alpha();

        Color color() const { return Color(m_rgbColor); }

    private:
        RGBColor(unsigned rgbColor)
            : m_rgbColor(rgbColor)
        {
        }

        RGBA32 m_rgbColor;
    };

} // namespace WebCore

#endif // RGBColor_h
