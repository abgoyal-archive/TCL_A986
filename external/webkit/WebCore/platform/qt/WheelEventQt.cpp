

#include "config.h"
#include "PlatformWheelEvent.h"

#include "PlatformMouseEvent.h"
#include "Scrollbar.h"

#include <qapplication.h>
#include <QWheelEvent>
#include <QGraphicsSceneWheelEvent>

namespace WebCore {

void PlatformWheelEvent::applyDelta(int delta, Qt::Orientation orientation)
{
    if (orientation == Qt::Horizontal) {
        m_deltaX = (delta / 120.0f);
        m_deltaY = 0;
    } else {
        m_deltaX = 0;
        m_deltaY = (delta / 120.0f);
    }

    m_wheelTicksX = m_deltaX;
    m_wheelTicksY = m_deltaY;

    // Use the same single scroll step as QTextEdit
    // (in QTextEditPrivate::init [h,v]bar->setSingleStep)
    static const float cDefaultQtScrollStep = 20.f;
#ifndef QT_NO_WHEELEVENT
    m_deltaX *= QApplication::wheelScrollLines() * cDefaultQtScrollStep;
    m_deltaY *= QApplication::wheelScrollLines() * cDefaultQtScrollStep;
#endif
}

PlatformWheelEvent::PlatformWheelEvent(QGraphicsSceneWheelEvent* e)
#ifdef QT_NO_WHEELEVENT
{
    Q_UNUSED(e);
}
#else
    : m_position(e->pos().toPoint())
    , m_globalPosition(e->screenPos())
    , m_granularity(ScrollByPixelWheelEvent)
    , m_isAccepted(false)
    , m_shiftKey(e->modifiers() & Qt::ShiftModifier)
    , m_ctrlKey(e->modifiers() & Qt::ControlModifier)
    , m_altKey(e->modifiers() & Qt::AltModifier)
    , m_metaKey(e->modifiers() & Qt::MetaModifier)
{
    applyDelta(e->delta(), e->orientation());
}
#endif // QT_NO_WHEELEVENT

PlatformWheelEvent::PlatformWheelEvent(QWheelEvent* e)
#ifdef QT_NO_WHEELEVENT
{
    Q_UNUSED(e);
}
#else
    : m_position(e->pos())
    , m_globalPosition(e->globalPos())
    , m_granularity(ScrollByPixelWheelEvent)
    , m_isAccepted(false)
    , m_shiftKey(e->modifiers() & Qt::ShiftModifier)
    , m_ctrlKey(e->modifiers() & Qt::ControlModifier)
    , m_altKey(e->modifiers() & Qt::AltModifier)
    , m_metaKey(e->modifiers() & Qt::MetaModifier)
{
    applyDelta(e->delta(), e->orientation());
}
#endif // QT_NO_WHEELEVENT

} // namespace WebCore
