

#import "WebDynamicScrollBarsView.h"
#import <WebCore/WebCoreFrameView.h>

@interface WebDynamicScrollBarsView (WebInternal) <WebCoreFrameScrollView>

- (BOOL)allowsHorizontalScrolling;
- (BOOL)allowsVerticalScrolling;

- (void)setScrollingModes:(WebCore::ScrollbarMode)hMode vertical:(WebCore::ScrollbarMode)vMode andLock:(BOOL)lock;
- (void)scrollingModes:(WebCore::ScrollbarMode*)hMode vertical:(WebCore::ScrollbarMode*)vMode;

- (WebCore::ScrollbarMode)horizontalScrollingMode;
- (WebCore::ScrollbarMode)verticalScrollingMode;

- (void)setHorizontalScrollingMode:(WebCore::ScrollbarMode)mode andLock:(BOOL)lock;
- (void)setVerticalScrollingMode:(WebCore::ScrollbarMode)mode andLock:(BOOL)lock;

- (void)setHorizontalScrollingModeLocked:(BOOL)locked;
- (void)setVerticalScrollingModeLocked:(BOOL)locked;
- (void)setScrollingModesLocked:(BOOL)mode;

- (BOOL)horizontalScrollingModeLocked;
- (BOOL)verticalScrollingModeLocked;

- (void)updateScrollers;
- (void)setSuppressLayout:(BOOL)flag;

@end
