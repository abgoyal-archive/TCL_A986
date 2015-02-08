

#ifndef RenderScrollbarTheme_h
#define RenderScrollbarTheme_h

#include "ScrollbarThemeComposite.h"

namespace WebCore {

class PlatformMouseEvent;
class Scrollbar;
class ScrollView;

class RenderScrollbarTheme : public ScrollbarThemeComposite {
public:
    virtual ~RenderScrollbarTheme() {};
    
    virtual int scrollbarThickness(ScrollbarControlSize controlSize) { return ScrollbarTheme::nativeTheme()->scrollbarThickness(controlSize); }

    virtual ScrollbarButtonsPlacement buttonsPlacement() const { return ScrollbarTheme::nativeTheme()->buttonsPlacement(); }

    virtual bool supportsControlTints() const { return true; }

    virtual void paintScrollCorner(ScrollView*, GraphicsContext* context, const IntRect& cornerRect);

    virtual bool shouldCenterOnThumb(Scrollbar* scrollbar, const PlatformMouseEvent& event) { return ScrollbarTheme::nativeTheme()->shouldCenterOnThumb(scrollbar, event); }
    
    virtual double initialAutoscrollTimerDelay() { return ScrollbarTheme::nativeTheme()->initialAutoscrollTimerDelay(); }
    virtual double autoscrollTimerDelay() { return ScrollbarTheme::nativeTheme()->autoscrollTimerDelay(); }

    virtual void registerScrollbar(Scrollbar* scrollbar) { return ScrollbarTheme::nativeTheme()->registerScrollbar(scrollbar); }
    virtual void unregisterScrollbar(Scrollbar* scrollbar) { return ScrollbarTheme::nativeTheme()->unregisterScrollbar(scrollbar); }

    virtual int minimumThumbLength(Scrollbar*);

    void buttonSizesAlongTrackAxis(Scrollbar* scrollbar, int& beforeSize, int& afterSize);
    
    static RenderScrollbarTheme* renderScrollbarTheme();

protected:
    virtual bool hasButtons(Scrollbar*);
    virtual bool hasThumb(Scrollbar*);

    virtual IntRect backButtonRect(Scrollbar*, ScrollbarPart, bool painting = false);
    virtual IntRect forwardButtonRect(Scrollbar*, ScrollbarPart, bool painting = false);
    virtual IntRect trackRect(Scrollbar*, bool painting = false);
    
    virtual void paintScrollbarBackground(GraphicsContext*, Scrollbar*);    
    virtual void paintTrackBackground(GraphicsContext*, Scrollbar*, const IntRect&);
    virtual void paintTrackPiece(GraphicsContext*, Scrollbar*, const IntRect&, ScrollbarPart);
    virtual void paintButton(GraphicsContext*, Scrollbar*, const IntRect&, ScrollbarPart);
    virtual void paintThumb(GraphicsContext*, Scrollbar*, const IntRect&);

    virtual IntRect constrainTrackRectToTrackPieces(Scrollbar*, const IntRect&);
};

} // namespace WebCore

#endif // RenderScrollbarTheme_h
