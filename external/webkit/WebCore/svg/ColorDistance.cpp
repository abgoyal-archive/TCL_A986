

#include "config.h"
#if ENABLE(SVG)
#include "ColorDistance.h"
#include "Color.h"
#include <wtf/MathExtras.h>

namespace WebCore {

ColorDistance::ColorDistance()
    : m_redDiff(0)
    , m_greenDiff(0)
    , m_blueDiff(0)
{
}

ColorDistance::ColorDistance(const Color& fromColor, const Color& toColor)
    : m_redDiff(toColor.red() - fromColor.red())
    , m_greenDiff(toColor.green() - fromColor.green())
    , m_blueDiff(toColor.blue() - fromColor.blue())
{
}

ColorDistance::ColorDistance(int redDiff, int greenDiff, int blueDiff)
    : m_redDiff(redDiff)
    , m_greenDiff(greenDiff)
    , m_blueDiff(blueDiff)
{
}

static inline int clampColorValue(int v)
{
    if (v > 255)
        v = 255;
    else if (v < 0)
        v = 0;
    return v;
}

ColorDistance ColorDistance::scaledDistance(float scaleFactor) const
{
    return ColorDistance(static_cast<int>(scaleFactor * m_redDiff),
                         static_cast<int>(scaleFactor * m_greenDiff),
                         static_cast<int>(scaleFactor * m_blueDiff));
}

Color ColorDistance::addColorsAndClamp(const Color& first, const Color& second)
{
    return Color(clampColorValue(first.red() + second.red()),
                 clampColorValue(first.green() + second.green()),
                 clampColorValue(first.blue() + second.blue()));
}

Color ColorDistance::addToColorAndClamp(const Color& color) const
{
    return Color(clampColorValue(color.red() + m_redDiff),
                 clampColorValue(color.green() + m_greenDiff),
                 clampColorValue(color.blue() + m_blueDiff));
}

bool ColorDistance::isZero() const
{
    return (m_redDiff == 0 && m_blueDiff == 0 && m_greenDiff == 0);
}

float ColorDistance::distance() const
{
    // This is just a simple distance calculation, not respecting color spaces
    return sqrtf(m_redDiff * m_redDiff + m_blueDiff * m_blueDiff + m_greenDiff * m_greenDiff);
}

}

#endif
