

#ifndef ScrollbarThemeComposite_h
#define ScrollbarThemeComposite_h

#include "ScrollbarTheme.h"

namespace WebCore {

class ScrollbarThemeComposite : public ScrollbarTheme {
public:
    virtual bool paint(Scrollbar*, GraphicsContext* context, const IntRect& damageRect);

    virtual ScrollbarPart hitTest(Scrollbar*, const PlatformMouseEvent&);

    virtual void invalidatePart(Scrollbar*, ScrollbarPart);

    virtual int thumbPosition(Scrollbar*);
    virtual int thumbLength(Scrollbar*);
    virtual int trackPosition(Scrollbar*);
    virtual int trackLength(Scrollbar*);

    virtual void paintScrollCorner(ScrollView*, GraphicsContext*, const IntRect& cornerRect);

protected:
    virtual bool hasButtons(Scrollbar*) = 0;
    virtual bool hasThumb(Scrollbar*) = 0;

    virtual IntRect backButtonRect(Scrollbar*, ScrollbarPart, bool painting = false) = 0;
    virtual IntRect forwardButtonRect(Scrollbar*, ScrollbarPart, bool painting = false) = 0;
    virtual IntRect trackRect(Scrollbar*, bool painting = false) = 0;

    virtual void splitTrack(Scrollbar*, const IntRect& track, IntRect& startTrack, IntRect& thumb, IntRect& endTrack);
    
    virtual int minimumThumbLength(Scrollbar*);

    virtual void paintScrollbarBackground(GraphicsContext*, Scrollbar*) {}
    virtual void paintTrackBackground(GraphicsContext*, Scrollbar*, const IntRect&) {}
    virtual void paintTrackPiece(GraphicsContext*, Scrollbar*, const IntRect&, ScrollbarPart) {}
    virtual void paintButton(GraphicsContext*, Scrollbar*, const IntRect&, ScrollbarPart) {}
    virtual void paintThumb(GraphicsContext*, Scrollbar*, const IntRect&) {}
    virtual void paintTickmarks(GraphicsContext*, Scrollbar*, const IntRect&) {}

    virtual IntRect constrainTrackRectToTrackPieces(Scrollbar*, const IntRect& rect) { return rect; }
};

}
#endif
