

#ifndef MouseRelatedEvent_h
#define MouseRelatedEvent_h

#include "IntPoint.h"
#include "UIEventWithKeyState.h"

namespace WebCore {

    // Internal only: Helper class for what's common between mouse and wheel events.
    class MouseRelatedEvent : public UIEventWithKeyState {
    public:
        // Note that these values are adjusted to counter the effects of zoom, so that values
        // exposed via DOM APIs are invariant under zooming.
        int screenX() const { return m_screenX; }
        int screenY() const { return m_screenY; }
        int clientX() const { return m_clientX; }
        int clientY() const { return m_clientY; }
        int layerX() const { return m_layerX; }
        int layerY() const { return m_layerY; }
        int offsetX() const { return m_offsetX; }
        int offsetY() const { return m_offsetY; }
        bool isSimulated() const { return m_isSimulated; }
        virtual int pageX() const;
        virtual int pageY() const;
        int x() const;
        int y() const;

        // Page point in "absolute" coordinates (i.e. post-zoomed, page-relative coords,
        // usable with RenderObject::absoluteToLocal).
        IntPoint absoluteLocation() const { return m_absoluteLocation; }
        void setAbsoluteLocation(const IntPoint& p) { m_absoluteLocation = p; }
    
    protected:
        MouseRelatedEvent();
        MouseRelatedEvent(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<AbstractView>,
                          int detail, int screenX, int screenY, int pageX, int pageY,
                          bool ctrlKey, bool altKey, bool shiftKey, bool metaKey, bool isSimulated = false);

        void initCoordinates();
        void initCoordinates(int clientX, int clientY);
        virtual void receivedTarget();

        void computePageLocation();
        
        // Expose these so MouseEvent::initMouseEvent can set them.
        int m_screenX;
        int m_screenY;
        int m_clientX;
        int m_clientY;

    private:
        int m_pageX;
        int m_pageY;
        int m_layerX;
        int m_layerY;
        int m_offsetX;
        int m_offsetY;
        IntPoint m_absoluteLocation;
        bool m_isSimulated;
    };

} // namespace WebCore

#endif // MouseRelatedEvent_h
