

#include "ScrollTypes.h"

namespace WebCore {
    class Frame;
}

@protocol WebCoreFrameScrollView
- (void)setScrollingModes:(WebCore::ScrollbarMode)hMode vertical:(WebCore::ScrollbarMode)vMode andLock:(BOOL)lock;
- (void)scrollingModes:(WebCore::ScrollbarMode*)hMode vertical:(WebCore::ScrollbarMode*)vMode;
- (void)setScrollBarsSuppressed:(BOOL)suppressed repaintOnUnsuppress:(BOOL)repaint;
@end

@protocol WebCoreFrameView
- (WebCore::Frame*)_web_frame;
@end
