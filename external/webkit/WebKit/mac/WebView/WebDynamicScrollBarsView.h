

// This is a Private header (containing SPI), despite the fact that its name
// does not contain the word Private.

// This was once used by Safari, but has not been for a long time.

// FIXME: <rdar://problem/5898985> Mail currently expects this header to define WebCoreScrollbarAlwaysOn.
extern const int WebCoreScrollbarAlwaysOn;

@interface WebDynamicScrollBarsView : NSScrollView {
    int hScroll; // FIXME: Should be WebCore::ScrollbarMode if this was an ObjC++ header.
    int vScroll; // Ditto.
    BOOL hScrollModeLocked;
    BOOL vScrollModeLocked;
    BOOL suppressLayout;
    BOOL suppressScrollers;
    BOOL inUpdateScrollers;
    BOOL verticallyPinnedByPreviousWheelEvent;
    BOOL horizontallyPinnedByPreviousWheelEvent;
    unsigned inUpdateScrollersLayoutPass;
}

// This was originally added for Safari's benefit, but Safari has not used it for a long time.
// Perhaps it can be removed.
- (void)setAllowsHorizontalScrolling:(BOOL)flag;
@end
