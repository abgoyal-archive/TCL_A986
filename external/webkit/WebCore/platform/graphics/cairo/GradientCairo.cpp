

#include "config.h"
#include "Gradient.h"

#include "CSSParser.h"
#include "GraphicsContext.h"
#include <cairo.h>

namespace WebCore {

void Gradient::platformDestroy()
{
    if (m_gradient) {
        cairo_pattern_destroy(m_gradient);
        m_gradient = 0;
    }
}

cairo_pattern_t* Gradient::platformGradient()
{
    if (m_gradient)
        return m_gradient;

    if (m_radial)
        m_gradient = cairo_pattern_create_radial(m_p0.x(), m_p0.y(), m_r0, m_p1.x(), m_p1.y(), m_r1);
    else
        m_gradient = cairo_pattern_create_linear(m_p0.x(), m_p0.y(), m_p1.x(), m_p1.y());

    Vector<ColorStop>::iterator stopIterator = m_stops.begin();
    while (stopIterator != m_stops.end()) {
        cairo_pattern_add_color_stop_rgba(m_gradient, stopIterator->stop, stopIterator->red, stopIterator->green, stopIterator->blue, stopIterator->alpha);
        ++stopIterator;
    }

    switch (m_spreadMethod) {
    case SpreadMethodPad:
        cairo_pattern_set_extend(m_gradient, CAIRO_EXTEND_PAD);
        break;
    case SpreadMethodReflect:
        cairo_pattern_set_extend(m_gradient, CAIRO_EXTEND_REFLECT);
        break;
    case SpreadMethodRepeat:
        cairo_pattern_set_extend(m_gradient, CAIRO_EXTEND_REPEAT);
        break;
    }

    cairo_matrix_t matrix = m_gradientSpaceTransformation;
    cairo_matrix_invert(&matrix);
    cairo_pattern_set_matrix(m_gradient, &matrix);

    return m_gradient;
}

void Gradient::fill(GraphicsContext* context, const FloatRect& rect)
{
    cairo_t* cr = context->platformContext();

    context->save();
    cairo_set_source(cr, platformGradient());
    cairo_rectangle(cr, rect.x(), rect.y(), rect.width(), rect.height());
    cairo_fill(cr);
    context->restore();
}

} //namespace
