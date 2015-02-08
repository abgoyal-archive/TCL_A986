

#if ENABLE(VIDEO)

#import <wtf/RefPtr.h>

namespace WebCore {
    class HTMLMediaElement;
}

@protocol WebVideoFullscreenControllerDelegate;
@class WebVideoFullscreenHUDWindowController;
@class WebWindowFadeAnimation;

@interface WebVideoFullscreenController : NSWindowController {
@private
    RefPtr<WebCore::HTMLMediaElement> _mediaElement; // (retain)
    id <WebVideoFullscreenControllerDelegate> _delegate; // (assign)

    NSWindow *_backgroundFullscreenWindow; // (retain)
    WebVideoFullscreenHUDWindowController *_hudController; // (retain)

    WebWindowFadeAnimation *_fadeAnimation; // (retain)

    BOOL _isEndingFullscreen;
    BOOL _isWindowLoaded;
    BOOL _forceDisableAnimation;
    SystemUIMode _savedUIMode;
    SystemUIOptions _savedUIOptions;
}

- (id <WebVideoFullscreenControllerDelegate>)delegate;
- (void)setDelegate:(id <WebVideoFullscreenControllerDelegate>)delegate;

- (void)setMediaElement:(WebCore::HTMLMediaElement*)mediaElement;
- (WebCore::HTMLMediaElement*)mediaElement;

- (void)enterFullscreen:(NSScreen *)screen;
- (void)exitFullscreen;

@end

#endif // ENABLE(VIDEO)
