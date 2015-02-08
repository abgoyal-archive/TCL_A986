

#include "config.h"
#include "Gradient.h"

#include "CSSParser.h"
#include "GraphicsContext.h"

#include <QGradient>
#include <QPainter>

namespace WebCore {

void Gradient::platformDestroy()
{
    delete m_gradient;
    m_gradient = 0;
}

QGradient* Gradient::platformGradient()
{
    if (m_gradient)
        return m_gradient;

    if (m_radial)
        m_gradient = new QRadialGradient(m_p1.x(), m_p1.y(), m_r1, m_p0.x(), m_p0.y());
    else
        m_gradient = new QLinearGradient(m_p0.x(), m_p0.y(), m_p1.x(), m_p1.y());

    QColor stopColor;
    Vector<ColorStop>::iterator stopIterator = m_stops.begin();
    qreal lastStop(0.0);
    const qreal lastStopDiff = 0.0000001;
    while (stopIterator != m_stops.end()) {
        stopColor.setRgbF(stopIterator->red, stopIterator->green, stopIterator->blue, stopIterator->alpha);
        if (qFuzzyCompare(lastStop, qreal(stopIterator->stop)))
            lastStop = stopIterator->stop + lastStopDiff;
        else
            lastStop = stopIterator->stop;
        if (m_radial && m_r0)
            lastStop = m_r0 / m_r1 + lastStop * (1.0f - m_r0 / m_r1);
        m_gradient->setColorAt(lastStop, stopColor);
        ++stopIterator;
    }

    switch (m_spreadMethod) {
    case SpreadMethodPad:
        m_gradient->setSpread(QGradient::PadSpread);
        break;
    case SpreadMethodReflect:
        m_gradient->setSpread(QGradient::ReflectSpread);
        break;
    case SpreadMethodRepeat:
        m_gradient->setSpread(QGradient::RepeatSpread);
        break;
    }

    return m_gradient;
}

void Gradient::fill(GraphicsContext* context, const FloatRect& rect)
{
    context->platformContext()->fillRect(rect, *platformGradient());
}

} //namespace
