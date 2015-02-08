

#include "config.h"
#include "Color.h"

#include <InterfaceDefs.h>


namespace WebCore {

Color::Color(const rgb_color& color)
    : m_color(makeRGBA(color.red, color.green, color.blue, color.alpha))
    , m_valid(true)
{
}

Color::operator rgb_color() const
{
    return make_color(red(), green(), blue(), alpha());
}


Color focusRingColor()
{
    return Color(keyboard_navigation_color());
}

} // namespace WebCore

