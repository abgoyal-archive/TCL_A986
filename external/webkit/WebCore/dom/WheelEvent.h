

#ifndef WheelEvent_h
#define WheelEvent_h

#include "MouseRelatedEvent.h"

namespace WebCore {

    // extension: mouse wheel event
    class WheelEvent : public MouseRelatedEvent {
    public:
        static PassRefPtr<WheelEvent> create()
        {
            return adoptRef(new WheelEvent);
        }
        static PassRefPtr<WheelEvent> create(float wheelTicksX, float wheelTicksY, PassRefPtr<AbstractView> view,
            int screenX, int screenY, int pageX, int pageY,
            bool ctrlKey, bool altKey, bool shiftKey, bool metaKey)
        {
            return adoptRef(new WheelEvent(wheelTicksX, wheelTicksY, view, screenX, screenY, pageX, pageY,
                ctrlKey, altKey, shiftKey, metaKey));
        }

        void initWheelEvent(int wheelDeltaX, int wheelDeltaY, PassRefPtr<AbstractView>,
                            int screenX, int screenY, int pageX, int pageY,
                            bool ctrlKey, bool altKey, bool shiftKey, bool metaKey);

        int wheelDelta() const { if (m_wheelDeltaY == 0) return m_wheelDeltaX; return m_wheelDeltaY; }
        int wheelDeltaX() const { return m_wheelDeltaX; }
        int wheelDeltaY() const { return m_wheelDeltaY; }

        // Needed for Objective-C legacy support
        bool isHorizontal() const { return m_wheelDeltaX; }

    private:
        WheelEvent();
        WheelEvent(float wheelTicksX, float wheelTicksY, PassRefPtr<AbstractView>,
                   int screenX, int screenY, int pageX, int pageY,
                   bool ctrlKey, bool altKey, bool shiftKey, bool metaKey);

        virtual bool isWheelEvent() const;

        int m_wheelDeltaX;
        int m_wheelDeltaY;
    };

} // namespace WebCore

#endif // WheelEvent_h
