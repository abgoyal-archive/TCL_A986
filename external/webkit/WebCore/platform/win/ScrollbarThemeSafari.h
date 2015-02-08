

#ifndef ScrollbarThemeSafari_h
#define ScrollbarThemeSafari_h

#if USE(SAFARI_THEME)

#include "ScrollbarThemeComposite.h"

namespace WebCore {

class ScrollbarThemeSafari : public ScrollbarThemeComposite {
public:
    virtual ~ScrollbarThemeSafari();

    virtual int scrollbarThickness(ScrollbarControlSize = RegularScrollbar);
    
    virtual bool supportsControlTints() const { return true; }

protected:
    virtual bool hasButtons(Scrollbar*);
    virtual bool hasThumb(Scrollbar*);

    virtual IntRect backButtonRect(Scrollbar*, ScrollbarPart, bool painting = false);
    virtual IntRect forwardButtonRect(Scrollbar*, ScrollbarPart, bool painting = false);
    virtual IntRect trackRect(Scrollbar*, bool painting = false);

    virtual int minimumThumbLength(Scrollbar*);
    
    virtual bool shouldCenterOnThumb(Scrollbar*, const PlatformMouseEvent&);

    virtual void paintTrackBackground(GraphicsContext*, Scrollbar*, const IntRect&);
    virtual void paintButton(GraphicsContext*, Scrollbar*, const IntRect&, ScrollbarPart);
    virtual void paintThumb(GraphicsContext*, Scrollbar*, const IntRect&);
};

}
#endif

#endif
