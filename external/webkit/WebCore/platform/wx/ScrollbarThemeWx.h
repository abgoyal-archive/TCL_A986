

#ifndef ScrollbarThemeWx_h
#define ScrollbarThemeWx_h

#include "ScrollbarTheme.h"
#include "ScrollbarThemeComposite.h"

namespace WebCore {

class ScrollbarThemeWx : public ScrollbarThemeComposite {
public:
    virtual ~ScrollbarThemeWx();
    virtual int scrollbarThickness(ScrollbarControlSize = RegularScrollbar);
    virtual bool paint(Scrollbar*, GraphicsContext*, const IntRect&);

    virtual void paintScrollCorner(ScrollView*, GraphicsContext*, const IntRect& cornerRect);
    
protected:
    virtual bool hasButtons(Scrollbar*) { return true; }
    virtual bool hasThumb(Scrollbar*);

    virtual IntSize buttonSize(Scrollbar*);

    virtual IntRect backButtonRect(Scrollbar*, ScrollbarPart, bool painting = false);
    virtual IntRect forwardButtonRect(Scrollbar*, ScrollbarPart, bool painting = false);
    virtual IntRect trackRect(Scrollbar*, bool painting = false);

    virtual void splitTrack(Scrollbar*, const IntRect& track, IntRect& startTrack, IntRect& thumb, IntRect& endTrack);
    
    virtual int minimumThumbLength(Scrollbar*);
};

}
#endif
