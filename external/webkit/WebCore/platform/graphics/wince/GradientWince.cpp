


#include "config.h"
#include "Gradient.h"

#include "GraphicsContext.h"

namespace WebCore {

void Gradient::platformDestroy()
{
}

static inline bool compareStops(const Gradient::ColorStop& a, const Gradient::ColorStop& b)
{
    return a.stop < b.stop;
}

const Vector<Gradient::ColorStop>& Gradient::getStops() const
{
    if (!m_stopsSorted) {
        if (m_stops.size())
            std::stable_sort(m_stops.begin(), m_stops.end(), compareStops);
        m_stopsSorted = true;
    }
    return m_stops;
}

void Gradient::fill(GraphicsContext* c, const FloatRect& r)
{
    c->fillRect(r, this);
}

}
