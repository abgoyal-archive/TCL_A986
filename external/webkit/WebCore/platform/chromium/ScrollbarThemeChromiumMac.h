

#ifndef ScrollbarThemeChromiumMac_h
#define ScrollbarThemeChromiumMac_h

#include "ScrollbarThemeComposite.h"

// This file (and its associated .mm file) is a clone of ScrollbarThemeMac.h.
// See the .mm file for details.

namespace WebCore {

class ScrollbarThemeChromiumMac : public ScrollbarThemeComposite {
public:
    ScrollbarThemeChromiumMac();
    virtual ~ScrollbarThemeChromiumMac();

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

    virtual int minimumThumbLength(Scrollbar*);

    virtual bool shouldCenterOnThumb(Scrollbar*, const PlatformMouseEvent&);

public:
    void preferencesChanged();
};

}

#endif // ScrollbarThemeChromiumMac_h
