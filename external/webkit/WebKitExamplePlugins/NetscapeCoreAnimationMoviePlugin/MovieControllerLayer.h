


#import <QuartzCore/QuartzCore.h>

@class QTMovie;

@interface MovieControllerLayer : CALayer {
    CGImageRef _playImage;
    CGImageRef _pauseImage;
    
    CGImageRef _sliderTrackLeft;
    CGImageRef _sliderTrackRight;
    CGImageRef _sliderTrackCenter;
    
    CGImageRef _thumb;
    
    QTMovie *_movie;
    NSTimer *_updateTimeTimer;
    
    BOOL _wasPlayingBeforeMouseDown;
    BOOL _isScrubbing;
    CGFloat _mouseDownXDelta;
}

- (void)setMovie:(QTMovie *)movie;

- (void)handleMouseDown:(CGPoint)point;
- (void)handleMouseUp:(CGPoint)point;
- (void)handleMouseDragged:(CGPoint)point;

@end
