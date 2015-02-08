

#if ENABLE(VIDEO)

namespace WebCore {
    class HTMLMediaElement;
}

@protocol WebVideoFullscreenHUDWindowControllerDelegate;

@interface WebVideoFullscreenHUDWindowController : NSWindowController
{
    id <WebVideoFullscreenHUDWindowControllerDelegate> _delegate;
    NSTimer *_timelineUpdateTimer;
#if !defined(BUILDING_ON_TIGER)
    NSTrackingArea *_area;
#endif
    BOOL _mouseIsInHUD;
    BOOL _isEndingFullscreen;
    BOOL _isScrubbing;

    NSControl *_timeline;
    NSTextField *_remainingTimeText;
    NSTextField *_elapsedTimeText;
    NSControl *_volumeSlider;
    NSButton *_playButton;
}

- (id <WebVideoFullscreenHUDWindowControllerDelegate>)delegate;
- (void)setDelegate:(id <WebVideoFullscreenHUDWindowControllerDelegate>)delegate;
- (void)fadeWindowIn;
- (void)fadeWindowOut;
- (void)closeWindow;
- (void)updateRate;

@end

@protocol WebVideoFullscreenHUDWindowControllerDelegate <NSObject>
- (void)requestExitFullscreen;
- (WebCore::HTMLMediaElement*)mediaElement;
@end

#endif
