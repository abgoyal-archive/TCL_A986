

#ifndef ScrollbarThemeChromiumLinux_h
#define ScrollbarThemeChromiumLinux_h

#include "ScrollbarThemeChromium.h"

namespace WebCore {
    class ScrollbarThemeChromiumLinux : public ScrollbarThemeChromium {
    public:
        virtual int scrollbarThickness(ScrollbarControlSize);

    protected:
        virtual void paintTrackPiece(GraphicsContext*, Scrollbar*, const IntRect&, ScrollbarPart);
        virtual void paintButton(GraphicsContext*, Scrollbar*, const IntRect&, ScrollbarPart);
        virtual void paintThumb(GraphicsContext*, Scrollbar*, const IntRect&);
        virtual bool shouldCenterOnThumb(Scrollbar*, const PlatformMouseEvent&);
        virtual IntSize buttonSize(Scrollbar*);
        virtual int minimumThumbLength(Scrollbar*);
    };
} // namespace WebCore

#endif
