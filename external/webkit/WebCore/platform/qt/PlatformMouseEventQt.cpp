

#include "config.h"
#include "PlatformMouseEvent.h"

#include <wtf/CurrentTime.h>

#include <QMouseEvent>
#include <QGraphicsSceneMouseEvent>

namespace WebCore {

PlatformMouseEvent::PlatformMouseEvent(QGraphicsSceneMouseEvent* event, int clickCount)
{
    m_timestamp = WTF::currentTime();

    switch (event->type()) {
    case QEvent::GraphicsSceneMouseDoubleClick:
    case QEvent::GraphicsSceneMousePress:
        m_eventType = MouseEventPressed;
        break;
    case QEvent::GraphicsSceneMouseRelease:
        m_eventType = MouseEventReleased;
        break;
    case QEvent::GraphicsSceneMouseMove:
    default:
        m_eventType = MouseEventMoved;
    }

    m_position = IntPoint(event->pos().toPoint());
    m_globalPosition = IntPoint(event->screenPos());

    if (event->button() == Qt::LeftButton || (event->buttons() & Qt::LeftButton))
        m_button = LeftButton;
    else if (event->button() == Qt::RightButton || (event->buttons() & Qt::RightButton))
        m_button = RightButton;
    else if (event->button() == Qt::MidButton || (event->buttons() & Qt::MidButton))
        m_button = MiddleButton;
    else
        m_button = NoButton;

    m_clickCount = clickCount;
    m_shiftKey =  (event->modifiers() & Qt::ShiftModifier) != 0;
    m_ctrlKey = (event->modifiers() & Qt::ControlModifier) != 0;
    m_altKey =  (event->modifiers() & Qt::AltModifier) != 0;
    m_metaKey = (event->modifiers() & Qt::MetaModifier) != 0;
}

PlatformMouseEvent::PlatformMouseEvent(QInputEvent* event, int clickCount)
{
    m_timestamp = WTF::currentTime();

    QMouseEvent* me = 0;

    switch (event->type()) {
    case QEvent::MouseMove:
        m_eventType = MouseEventMoved;
        me = static_cast<QMouseEvent *>(event);
        break;
    case QEvent::MouseButtonDblClick:
    case QEvent::MouseButtonPress:
        m_eventType = MouseEventPressed;
        me = static_cast<QMouseEvent *>(event);
        break;
    case QEvent::MouseButtonRelease:
        m_eventType = MouseEventReleased;
        me = static_cast<QMouseEvent *>(event);
        break;
#ifndef QT_NO_CONTEXTMENU
    case QEvent::ContextMenu: {
        m_eventType = MouseEventPressed;
        QContextMenuEvent *ce = static_cast<QContextMenuEvent *>(event);
        m_position = IntPoint(ce->pos());
        m_globalPosition = IntPoint(ce->globalPos());
        m_button = RightButton;
        break;
    }
#endif // QT_NO_CONTEXTMENU
    default:
        m_eventType = MouseEventMoved;
    }

    if (me) {
        m_position = IntPoint(me->pos());
        m_globalPosition = IntPoint(me->globalPos());

        if (me->button() == Qt::LeftButton || (me->buttons() & Qt::LeftButton))
            m_button = LeftButton;
        else if (me->button() == Qt::RightButton || (me->buttons() & Qt::RightButton))
            m_button = RightButton;
        else if (me->button() == Qt::MidButton || (me->buttons() & Qt::MidButton))
            m_button = MiddleButton;
        else
            m_button = NoButton;
    }

    m_clickCount = clickCount;
    m_shiftKey =  (event->modifiers() & Qt::ShiftModifier) != 0;
    m_ctrlKey = (event->modifiers() & Qt::ControlModifier) != 0;
    m_altKey =  (event->modifiers() & Qt::AltModifier) != 0;
    m_metaKey = (event->modifiers() & Qt::MetaModifier) != 0;
}

}

// vim: ts=4 sw=4 et
