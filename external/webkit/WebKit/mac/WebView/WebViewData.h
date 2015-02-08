

#import "WebTypesInternal.h"
#import "WebDelegateImplementationCaching.h"
#import <WebCore/PlatformString.h>
#import <WebCore/WebCoreKeyboardUIMode.h>
#import <wtf/HashMap.h>
#import <wtf/RetainPtr.h>

namespace WebCore {
    class Page;
}

@class WebInspector;
@class WebNodeHighlight;
@class WebPluginDatabase;
@class WebPreferences;
@class WebTextCompletionController;
@protocol WebFormDelegate;
@protocol WebGeolocationProvider;
#if ENABLE(VIDEO)
@class WebVideoFullscreenController;
#endif

extern BOOL applicationIsTerminating;
extern int pluginDatabaseClientCount;

// FIXME: This should be renamed to WebViewData.
@interface WebViewPrivate : NSObject {
@public
    WebCore::Page* page;
    
    id UIDelegate;
    id UIDelegateForwarder;
    id resourceProgressDelegate;
    id downloadDelegate;
    id policyDelegate;
    id policyDelegateForwarder;
    id frameLoadDelegate;
    id frameLoadDelegateForwarder;
    id <WebFormDelegate> formDelegate;
    id editingDelegate;
    id editingDelegateForwarder;
    id scriptDebugDelegate;
    id historyDelegate;

    WebInspector *inspector;
    WebNodeHighlight *currentNodeHighlight;

    BOOL allowsUndo;
        
    float zoomMultiplier;

    NSString *applicationNameForUserAgent;
    WebCore::String userAgent;
    BOOL userAgentOverridden;
    
    WebPreferences *preferences;
    BOOL useSiteSpecificSpoofing;

    NSWindow *hostWindow;

    int programmaticFocusCount;
    
    WebResourceDelegateImplementationCache resourceLoadDelegateImplementations;
    WebFrameLoadDelegateImplementationCache frameLoadDelegateImplementations;
    WebScriptDebugDelegateImplementationCache scriptDebugDelegateImplementations;
    WebHistoryDelegateImplementationCache historyDelegateImplementations;

    void *observationInfo;
    
    BOOL closed;
    BOOL shouldCloseWithWindow;
    BOOL mainFrameDocumentReady;
    BOOL drawsBackground;
    BOOL editable;
    BOOL tabKeyCyclesThroughElementsChanged;
    BOOL becomingFirstResponder;
    BOOL becomingFirstResponderFromOutside;
    BOOL hoverFeedbackSuspended;
    BOOL usesPageCache;
    BOOL catchesDelegateExceptions;
    BOOL cssAnimationsSuspended;

    NSColor *backgroundColor;

    NSString *mediaStyle;
    
    BOOL hasSpellCheckerDocumentTag;
    NSInteger spellCheckerDocumentTag;

    BOOL smartInsertDeleteEnabled;
    BOOL selectTrailingWhitespaceEnabled;
        
#if ENABLE(DASHBOARD_SUPPORT)
    BOOL dashboardBehaviorAlwaysSendMouseEventsToAllWindows;
    BOOL dashboardBehaviorAlwaysSendActiveNullEventsToPlugIns;
    BOOL dashboardBehaviorAlwaysAcceptsFirstMouse;
    BOOL dashboardBehaviorAllowWheelScrolling;
#endif
    
    // WebKit has both a global plug-in database and a separate, per WebView plug-in database. Dashboard uses the per WebView database.
    WebPluginDatabase *pluginDatabase;
    
    HashMap<unsigned long, RetainPtr<id> > identifierMap;

    BOOL _keyboardUIModeAccessed;
    WebCore::KeyboardUIMode _keyboardUIMode;

    BOOL shouldUpdateWhileOffscreen;
    
    // When this flag is set, we will not make any subviews underneath this WebView.  This means no WebFrameViews and no WebHTMLViews.
    BOOL usesDocumentViews;
    
#if USE(ACCELERATED_COMPOSITING)
    // When this flag is set, next time a WebHTMLView draws, it needs to temporarily disable screen updates
    // so that the NSView drawing is visually synchronized with CALayer updates.
    BOOL needsOneShotDrawingSynchronization;
    BOOL postsAcceleratedCompositingNotifications;
    // Run loop observer used to implement the compositing equivalent of -viewWillDraw
    CFRunLoopObserverRef layerSyncRunLoopObserver;
#endif

    NSPasteboard *insertionPasteboard;
            
    NSSize lastLayoutSize;

    BOOL ignoringMouseDraggedEvents;

    NSEvent *mouseDownEvent; // Kept after handling the event.
    BOOL handlingMouseDownEvent;
    NSEvent *keyDownEvent; // Kept after handling the event.

    WebTextCompletionController *completionController;

    NSTimer *autoscrollTimer;
    NSEvent *autoscrollTriggerEvent;

    CFRunLoopTimerRef updateMouseoverTimer;
#if ENABLE(VIDEO)
    WebVideoFullscreenController *fullscreenController;
#endif

    id<WebGeolocationProvider> _geolocationProvider;
}
@end
