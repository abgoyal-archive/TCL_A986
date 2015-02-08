

#include "config.h"
#include "Color.h"

#if PLATFORM(CG)

#include <wtf/Assertions.h>
#include <wtf/RetainPtr.h>
#include <ApplicationServices/ApplicationServices.h>

namespace WebCore {

Color::Color(CGColorRef color)
{
    if (!color) {
        m_color = 0;
        m_valid = false;
        return;
    }

    size_t numComponents = CGColorGetNumberOfComponents(color);
    const CGFloat* components = CGColorGetComponents(color);

    float r = 0;
    float g = 0;
    float b = 0;
    float a = 0;

    switch (numComponents) {
    case 2:
        r = g = b = components[0];
        a = components[1];
        break;
    case 4:
        r = components[0];
        g = components[1];
        b = components[2];
        a = components[3];
        break;
    default:
        ASSERT_NOT_REACHED();
    }

    m_color = makeRGBA(r * 255, g * 255, b * 255, a * 255);
}

#if OS(WINDOWS)

CGColorRef createCGColor(const Color& c)
{
    CGColorRef color = NULL;
    CMProfileRef prof = NULL;
    CMGetSystemProfile(&prof);

    RetainPtr<CGColorSpaceRef> rgbSpace(AdoptCF, CGColorSpaceCreateWithPlatformColorSpace(prof));

    if (rgbSpace) {
        CGFloat components[4] = { static_cast<CGFloat>(c.red()) / 255, static_cast<CGFloat>(c.green()) / 255,
                                  static_cast<CGFloat>(c.blue()) / 255, static_cast<CGFloat>(c.alpha()) / 255 };
        color = CGColorCreate(rgbSpace.get(), components);
    }

    CMCloseProfile(prof);

    return color;
}

#endif // OS(WINDOWS)

}

#endif // PLATFORM(CG)
