

#include "config.h"
#include "Pen.h"

namespace WebCore {

Pen::Pen(const Color &color, unsigned width, PenStyle style) : m_style(style), m_width(width), m_color(color)
{
}

const Color &Pen::color() const
{
    return m_color;
}

unsigned Pen::width() const
{
    return m_width;
}

Pen::PenStyle Pen::style() const
{
    return m_style;
}

void Pen::setColor(const Color &color)
{
    m_color = color;
}

void Pen::setWidth(unsigned width)
{
    m_width = width;
}

void Pen::setStyle(PenStyle style)
{
    m_style = style;
}

bool Pen::operator==(const Pen &compareTo) const
{
    return (m_width == compareTo.m_width) &&
        (m_style == compareTo.m_style) &&
        (m_color == compareTo.m_color);
}

bool Pen::operator!=(const Pen &compareTo) const
{
    return !(*this == compareTo);
}

}
