

#ifndef ScrollTypes_h
#define ScrollTypes_h

namespace WebCore {

    enum ScrollDirection {
        ScrollUp,
        ScrollDown,
        ScrollLeft,
        ScrollRight
    };

    enum ScrollGranularity {
        ScrollByLine,
        ScrollByPage,
        ScrollByDocument,
        ScrollByPixel
    };

    enum ScrollbarOrientation { HorizontalScrollbar, VerticalScrollbar };

    enum ScrollbarMode { ScrollbarAuto, ScrollbarAlwaysOff, ScrollbarAlwaysOn };

    enum ScrollbarControlSize { RegularScrollbar, SmallScrollbar };

    typedef unsigned ScrollbarControlState;

    enum ScrollbarControlStateMask {
        ActiveScrollbarState = 1,
        EnabledScrollbarState = 1 << 1,
        PressedScrollbarState = 1 << 2,
    };

    enum ScrollbarPart {
        NoPart = 0,
        BackButtonStartPart = 1,
        ForwardButtonStartPart = 1 << 1,
        BackTrackPart = 1 << 2,
        ThumbPart = 1 << 3,
        ForwardTrackPart = 1 << 4,
        BackButtonEndPart = 1 << 5,
        ForwardButtonEndPart = 1 << 6,
        ScrollbarBGPart = 1 << 7,
        TrackBGPart = 1 << 8,
        AllParts = 0xffffffff,
    };

    enum ScrollbarButtonsPlacement {
        ScrollbarButtonsNone,
        ScrollbarButtonsSingle,
        ScrollbarButtonsDoubleStart,
        ScrollbarButtonsDoubleEnd,
        ScrollbarButtonsDoubleBoth
    };
    
    typedef unsigned ScrollbarControlPartMask;

}

#endif
