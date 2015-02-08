

#include "config.h"
#include "Color.h"

#include <QColor>

namespace WebCore {

Color::Color(const QColor& c)
    : m_color(makeRGBA(c.red(), c.green(), c.blue(), c.alpha()))
{
    m_valid = c.isValid();
}

Color::operator QColor() const
{
    if (m_valid)
        return QColor(red(), green(), blue(), alpha());
    else
        return QColor();
}

}

// vim: ts=4 sw=4 et
