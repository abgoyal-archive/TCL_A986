

#ifndef ScrollbarThemeMac_h
#define ScrollbarThemeMac_h

#include "ScrollbarThemeComposite.h"

namespace WebCore {

class ScrollbarThemeMac : public ScrollbarThemeComposite {
public:
    ScrollbarThemeMac();
    virtual ~ScrollbarThemeMac();

    virtual bool paint(Scrollbar*, GraphicsContext* context, const IntRect& damageRect);

    virtual int scrollbarThickness(ScrollbarControlSize = RegularScrollbar);
    
    virtual bool supportsControlTints() const { return true; }

    virtual double initialAutoscrollTimerDelay();
    virtual double autoscrollTimerDelay();

    virtual ScrollbarButtonsPlacement buttonsPlacement() const;

    virtual void registerScrollbar(Scrollbar*);
    virtual void unregisterScrollbar(Scrollbar*);

protected:
    virtual bool hasButtons(Scrollbar*);
    virtual bool hasThumb(Scrollbar*);

    virtual IntRect backButtonRect(Scrollbar*, ScrollbarPart, bool painting = false);
    virtual IntRect forwardButtonRect(Scrollbar*, ScrollbarPart, bool painting = false);
    virtual IntRect trackRect(Scrollbar*, bool painting = false);

    virtual int maxOverlapBetweenPages() { return 40; }

    virtual int minimumThumbLength(Scrollbar*);
    
    virtual bool shouldCenterOnThumb(Scrollbar*, const PlatformMouseEvent&);
    
public:
    void preferencesChanged();
};

}

#endif
