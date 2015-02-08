

#ifndef ScrollbarThemeWin_h
#define ScrollbarThemeWin_h

#include "ScrollbarThemeComposite.h"

namespace WebCore {

class ScrollbarThemeWin : public ScrollbarThemeComposite {
public:
    ScrollbarThemeWin();
    virtual ~ScrollbarThemeWin();

    virtual int scrollbarThickness(ScrollbarControlSize = RegularScrollbar);

    virtual void themeChanged();
    
    virtual bool invalidateOnMouseEnterExit();

protected:
    virtual bool hasButtons(Scrollbar*) { return true; }
    virtual bool hasThumb(Scrollbar*);

    virtual IntRect backButtonRect(Scrollbar*, ScrollbarPart, bool painting = false);
    virtual IntRect forwardButtonRect(Scrollbar*, ScrollbarPart, bool painting = false);
    virtual IntRect trackRect(Scrollbar*, bool painting = false);

    virtual bool shouldCenterOnThumb(Scrollbar*, const PlatformMouseEvent&);
    virtual bool shouldSnapBackToDragOrigin(Scrollbar*, const PlatformMouseEvent&);

    virtual void paintTrackBackground(GraphicsContext*, Scrollbar*, const IntRect&);
    virtual void paintTrackPiece(GraphicsContext*, Scrollbar*, const IntRect&, ScrollbarPart);
    virtual void paintButton(GraphicsContext*, Scrollbar*, const IntRect&, ScrollbarPart);
    virtual void paintThumb(GraphicsContext*, Scrollbar*, const IntRect&);
};

}
#endif
