

#include "config.h"
#include "Color.h"

#include <gdk/gdk.h>

namespace WebCore {

Color::Color(const GdkColor& c)
    : m_color(makeRGB(c.red >> 8, c.green >> 8, c.blue >> 8))
    , m_valid(true)
{
}

}

// vim: ts=4 sw=4 et
