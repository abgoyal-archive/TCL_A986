

#import <Cocoa/Cocoa.h>

@interface WebWindowScaleAnimation : NSAnimation {
@private
    NSRect _initialFrame, _finalFrame, _realFrame;
    NSWindow *_window; // (assign)
    NSAnimation *_subAnimation; // (retain)
    NSTimeInterval _hintedDuration;
}
- (id)initWithHintedDuration:(NSTimeInterval)duration window:(NSWindow *)window initalFrame:(NSRect)initialFrame finalFrame:(NSRect)finalFrame;

- (void)setSubAnimation:(NSAnimation *)animation;

- (NSRect)currentFrame;

// Be sure to call setWindow:nil to clear the weak link _window when appropriate
- (void)setWindow:(NSWindow *)window;
@end


@interface WebWindowFadeAnimation : NSAnimation {
@private
    CGFloat _initialAlpha, _finalAlpha;
    NSWindow *_window; // (assign)
    BOOL _isStopped;
    
}
- (id)initWithDuration:(NSTimeInterval)duration window:(NSWindow *)window initialAlpha:(CGFloat)initialAlpha finalAlpha:(CGFloat)finalAlpha;

- (CGFloat)currentAlpha;

// Be sure to call setWindow:nil to clear the weak link _window when appropriate
- (void)setWindow:(NSWindow *)window;
@end
