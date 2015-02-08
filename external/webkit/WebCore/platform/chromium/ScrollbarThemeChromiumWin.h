

#ifndef ScrollbarThemeChromiumWin_h
#define ScrollbarThemeChromiumWin_h

#include "ScrollbarThemeChromium.h"

namespace WebCore {
    class ScrollbarThemeChromiumWin : public ScrollbarThemeChromium {
    public:
        virtual int scrollbarThickness(ScrollbarControlSize);
        virtual bool invalidateOnMouseEnterExit();
        virtual bool shouldSnapBackToDragOrigin(Scrollbar*, const PlatformMouseEvent&);

    protected:
        virtual void paintTrackPiece(GraphicsContext*, Scrollbar*, const IntRect&, ScrollbarPart);
        virtual void paintButton(GraphicsContext*, Scrollbar*, const IntRect&, ScrollbarPart);
        virtual void paintThumb(GraphicsContext*, Scrollbar*, const IntRect&);
        virtual bool shouldCenterOnThumb(Scrollbar*, const PlatformMouseEvent&);
        virtual IntSize buttonSize(Scrollbar*);

    private:
        int getThemeState(Scrollbar*, ScrollbarPart) const;
        int getThemeArrowState(Scrollbar*, ScrollbarPart) const;
        int getClassicThemeState(Scrollbar*, ScrollbarPart) const;
    };
} // namespace WebCore

#endif
