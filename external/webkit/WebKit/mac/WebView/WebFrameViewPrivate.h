

#import <WebKit/WebFrameView.h>

@interface WebFrameView (WebPrivate)

- (WebFrameView *)_largestChildWithScrollBars;

- (BOOL)_hasScrollBars;

- (NSClipView *)_contentView;

- (Class)_customScrollViewClass;

- (void)_setCustomScrollViewClass:(Class)scrollViewClass;

@end
