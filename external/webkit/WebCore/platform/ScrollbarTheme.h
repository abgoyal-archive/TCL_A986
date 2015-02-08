

#ifndef ScrollbarTheme_h
#define ScrollbarTheme_h

#include "GraphicsContext.h"
#include "IntRect.h"
#include "ScrollTypes.h"

namespace WebCore {

class PlatformMouseEvent;
class Scrollbar;
class ScrollView;

class ScrollbarTheme : public Noncopyable {
public:
    virtual ~ScrollbarTheme() {};

    virtual bool paint(Scrollbar*, GraphicsContext*, const IntRect& /*damageRect*/) { return false; }
    virtual ScrollbarPart hitTest(Scrollbar*, const PlatformMouseEvent&) { return NoPart; }
    
    virtual int scrollbarThickness(ScrollbarControlSize = RegularScrollbar) { return 0; }

    virtual ScrollbarButtonsPlacement buttonsPlacement() const { return ScrollbarButtonsSingle; }

    virtual bool supportsControlTints() const { return false; }

    virtual void themeChanged() {}
    
    virtual bool invalidateOnMouseEnterExit() { return false; }

    void invalidateParts(Scrollbar* scrollbar, ScrollbarControlPartMask mask)
    {
        if (mask & BackButtonStartPart)
            invalidatePart(scrollbar, BackButtonStartPart);
        if (mask & ForwardButtonStartPart)
            invalidatePart(scrollbar, ForwardButtonStartPart);
        if (mask & BackTrackPart)
            invalidatePart(scrollbar, BackTrackPart);
        if (mask & ThumbPart)
            invalidatePart(scrollbar, ThumbPart);
        if (mask & ForwardTrackPart)
            invalidatePart(scrollbar, ForwardTrackPart);
        if (mask & BackButtonEndPart)
            invalidatePart(scrollbar, BackButtonEndPart);
        if (mask & ForwardButtonEndPart)
            invalidatePart(scrollbar, ForwardButtonEndPart);
    }

    virtual void invalidatePart(Scrollbar*, ScrollbarPart) {}

    virtual void paintScrollCorner(ScrollView*, GraphicsContext* context, const IntRect& cornerRect) { context->fillRect(cornerRect, Color::white, DeviceColorSpace); }

    virtual bool shouldCenterOnThumb(Scrollbar*, const PlatformMouseEvent&) { return false; }
    virtual bool shouldSnapBackToDragOrigin(Scrollbar*, const PlatformMouseEvent&) { return false; }
    virtual int thumbPosition(Scrollbar*) { return 0; } // The position of the thumb relative to the track.
    virtual int thumbLength(Scrollbar*) { return 0; } // The length of the thumb along the axis of the scrollbar.
    virtual int trackPosition(Scrollbar*) { return 0; } // The position of the track relative to the scrollbar.
    virtual int trackLength(Scrollbar*) { return 0; } // The length of the track along the axis of the scrollbar.

    virtual int maxOverlapBetweenPages() { return std::numeric_limits<int>::max(); }

    virtual double initialAutoscrollTimerDelay() { return 0.25; }
    virtual double autoscrollTimerDelay() { return 0.05; }

    virtual void registerScrollbar(Scrollbar*) {}
    virtual void unregisterScrollbar(Scrollbar*) {}

    static ScrollbarTheme* nativeTheme(); // Must be implemented to return the correct theme subclass.
};

}
#endif
