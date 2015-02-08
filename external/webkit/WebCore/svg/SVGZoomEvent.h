
 
#ifndef SVGZoomEvent_h
#define SVGZoomEvent_h
#if ENABLE(SVG)

#include "FloatRect.h"
#include "UIEvent.h"

namespace WebCore {

    class SVGZoomEvent : public UIEvent {
    public:
        static PassRefPtr<SVGZoomEvent> create() { return adoptRef(new SVGZoomEvent); }
        virtual ~SVGZoomEvent();

        // 'SVGZoomEvent' functions
        FloatRect zoomRectScreen() const;

        float previousScale() const;
        void setPreviousScale(float);

        FloatPoint previousTranslate() const;

        float newScale() const;
        void setNewScale(float);

        FloatPoint newTranslate() const;

        virtual bool isSVGZoomEvent() const;

    private:
        SVGZoomEvent();

        float m_newScale;
        float m_previousScale;
        
        FloatRect m_zoomRectScreen;
        
        FloatPoint m_newTranslate;
        FloatPoint m_previousTranslate;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGZoomEvent_h

// vim:ts=4:noet
