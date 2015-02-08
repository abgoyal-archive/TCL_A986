

#ifndef PlatformWheelEvent_h
#define PlatformWheelEvent_h

#include "IntPoint.h"

#if PLATFORM(GTK)
typedef struct _GdkEventScroll GdkEventScroll;
#endif

#if PLATFORM(QT)
QT_BEGIN_NAMESPACE
class QWheelEvent;
class QGraphicsSceneWheelEvent;
QT_END_NAMESPACE
#endif

#if PLATFORM(WIN)
typedef struct HWND__* HWND;
typedef unsigned WPARAM;
typedef long LPARAM;
#endif

#if PLATFORM(WX)
class wxMouseEvent;
class wxPoint;
#endif

#if PLATFORM(HAIKU)
class BMessage;
#endif

namespace WebCore {

    class FloatPoint;
    class FloatSize;

    // Wheel events come in two flavors:
    // The ScrollByPixelWheelEvent is a fine-grained event that specifies the precise number of pixels to scroll.  It is sent directly by MacBook touchpads on OS X,
    // and synthesized in other cases where platforms generate line-by-line scrolling events.
    // The ScrollByPageWheelEvent indicates that the wheel event should scroll an entire page.  In this case WebCore's built in paging behavior is used to page
    // up and down (you get the same behavior as if the user was clicking in a scrollbar track to page up or page down).  Page scrolling only works in the vertical direction.
    enum PlatformWheelEventGranularity { ScrollByPageWheelEvent, ScrollByPixelWheelEvent };
    
    class PlatformWheelEvent {
    public:
        PlatformWheelEvent()
            : m_deltaX(0)
            , m_deltaY(0)
            , m_wheelTicksX(0)
            , m_wheelTicksY(0)
            , m_granularity(ScrollByPixelWheelEvent)
            , m_isAccepted(false)
            , m_shiftKey(false)
            , m_ctrlKey(false)
            , m_altKey(false)
            , m_metaKey(false)
        {
        }

        const IntPoint& pos() const { return m_position; } // PlatformWindow coordinates.
        const IntPoint& globalPos() const { return m_globalPosition; } // Screen coordinates.

        float deltaX() const { return m_deltaX; }
        float deltaY() const { return m_deltaY; }

        float wheelTicksX() const { return m_wheelTicksX; }
        float wheelTicksY() const { return m_wheelTicksY; }

        PlatformWheelEventGranularity granularity() const { return m_granularity; }

        bool isAccepted() const { return m_isAccepted; }
        bool shiftKey() const { return m_shiftKey; }
        bool ctrlKey() const { return m_ctrlKey; }
        bool altKey() const { return m_altKey; }
        bool metaKey() const { return m_metaKey; }

        int x() const { return m_position.x(); } // PlatformWindow coordinates.
        int y() const { return m_position.y(); }
        int globalX() const { return m_globalPosition.x(); } // Screen coordinates.
        int globalY() const { return m_globalPosition.y(); }

        void accept() { m_isAccepted = true; }
        void ignore() { m_isAccepted = false; }

        void turnVerticalTicksIntoHorizontal()
        {
            m_deltaX = m_deltaY;
            m_deltaY = 0;

            m_wheelTicksX = m_wheelTicksY;
            m_wheelTicksY = 0;
        }

#if PLATFORM(GTK)
        PlatformWheelEvent(GdkEventScroll*);
#endif

#if PLATFORM(MAC) && defined(__OBJC__)
        PlatformWheelEvent(NSEvent *, NSView *windowView);
#endif

#if PLATFORM(QT)
        PlatformWheelEvent(QWheelEvent*);
        PlatformWheelEvent(QGraphicsSceneWheelEvent*);
        void applyDelta(int delta, Qt::Orientation);
#endif

#if PLATFORM(WIN)
        PlatformWheelEvent(HWND, WPARAM, LPARAM, bool isMouseHWheel);
        PlatformWheelEvent(HWND, const FloatSize& delta, const FloatPoint& location);
#endif

#if PLATFORM(WX)
        PlatformWheelEvent(const wxMouseEvent&, const wxPoint&);
#endif

#if PLATFORM(HAIKU)
        PlatformWheelEvent(BMessage*);
#endif

    protected:
        IntPoint m_position;
        IntPoint m_globalPosition;
        float m_deltaX;
        float m_deltaY;
        float m_wheelTicksX;
        float m_wheelTicksY;
        PlatformWheelEventGranularity m_granularity;
        bool m_isAccepted;
        bool m_shiftKey;
        bool m_ctrlKey;
        bool m_altKey;
        bool m_metaKey;
    };

} // namespace WebCore

#endif // PlatformWheelEvent_h
