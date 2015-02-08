

#include "config.h"
#include "Color.h"

#include <wx/defs.h>
#include <wx/colour.h>

namespace WebCore {

Color::Color(const wxColour& color)
{
    m_color = makeRGBA((int)color.Red(), (int)color.Green(), (int)color.Blue(), (int)color.Alpha());
}

Color::operator wxColour() const
{
    return wxColour(red(), green(), blue(), alpha());
}

}
