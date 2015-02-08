

#import <WebKit/WebView.h>
#import <WebKit/WebFramePrivate.h>

#if !defined(ENABLE_DASHBOARD_SUPPORT)
#define ENABLE_DASHBOARD_SUPPORT 1
#endif

#if MAC_OS_X_VERSION_MAX_ALLOWED <= MAC_OS_X_VERSION_10_4
#define WebNSInteger int
#define WebNSUInteger unsigned int
#else
#define WebNSInteger NSInteger
#define WebNSUInteger NSUInteger
#endif

@class NSError;
@class WebFrame;
@class WebGeolocationPosition;
@class WebInspector;
@class WebPreferences;
@class WebScriptWorld;
@class WebTextIterator;

@protocol WebFormDelegate;

extern NSString *_WebCanGoBackKey;
extern NSString *_WebCanGoForwardKey;
extern NSString *_WebEstimatedProgressKey;
extern NSString *_WebIsLoadingKey;
extern NSString *_WebMainFrameIconKey;
extern NSString *_WebMainFrameTitleKey;
extern NSString *_WebMainFrameURLKey;
extern NSString *_WebMainFrameDocumentKey;

// pending public WebElementDictionary keys
extern NSString *WebElementTitleKey;             // NSString of the title of the element (used by Safari)
extern NSString *WebElementSpellingToolTipKey;   // NSString of a tooltip representing misspelling or bad grammar (used internally)
extern NSString *WebElementIsContentEditableKey; // NSNumber indicating whether the inner non-shared node is content editable (used internally)

// other WebElementDictionary keys
extern NSString *WebElementLinkIsLiveKey;        // NSNumber of BOOL indictating whether the link is live or not
extern NSString *WebElementIsInScrollBarKey;

// One of the subviews of the WebView entered compositing mode.
extern NSString *_WebViewDidStartAcceleratedCompositingNotification;

#if ENABLE_DASHBOARD_SUPPORT
typedef enum {
    WebDashboardBehaviorAlwaysSendMouseEventsToAllWindows,
    WebDashboardBehaviorAlwaysSendActiveNullEventsToPlugIns,
    WebDashboardBehaviorAlwaysAcceptsFirstMouse,
    WebDashboardBehaviorAllowWheelScrolling,
    WebDashboardBehaviorUseBackwardCompatibilityMode
} WebDashboardBehavior;
#endif

typedef enum {
    WebInjectAtDocumentStart,
    WebInjectAtDocumentEnd,
} WebUserScriptInjectionTime;

@interface WebController : NSTreeController {
    IBOutlet WebView *webView;
}
- (WebView *)webView;
- (void)setWebView:(WebView *)newWebView;
@end

@interface WebView (WebViewEditingActionsPendingPublic)

- (void)outdent:(id)sender;

@end

@interface WebView (WebPendingPublic)

- (void)scheduleInRunLoop:(NSRunLoop *)runLoop forMode:(NSString *)mode;
- (void)unscheduleFromRunLoop:(NSRunLoop *)runLoop forMode:(NSString *)mode;

- (BOOL)searchFor:(NSString *)string direction:(BOOL)forward caseSensitive:(BOOL)caseFlag wrap:(BOOL)wrapFlag startInSelection:(BOOL)startInSelection;

- (void)setMainFrameDocumentReady:(BOOL)mainFrameDocumentReady;

- (void)setTabKeyCyclesThroughElements:(BOOL)cyclesElements;
- (BOOL)tabKeyCyclesThroughElements;

- (void)scrollDOMRangeToVisible:(DOMRange *)range;

// setHoverFeedbackSuspended: can be called by clients that want to temporarily prevent the webView
// from displaying feedback about mouse position. Each WebDocumentView class that displays feedback
// about mouse position should honor this setting.
- (void)setHoverFeedbackSuspended:(BOOL)newValue;
- (BOOL)isHoverFeedbackSuspended;

- (void)setScriptDebugDelegate:(id)delegate;

- (id)scriptDebugDelegate;

- (void)setHistoryDelegate:(id)delegate;

- (id)historyDelegate;

- (BOOL)shouldClose;

- (NSAppleEventDescriptor *)aeDescByEvaluatingJavaScriptFromString:(NSString *)script;

// Support for displaying multiple text matches.
// These methods might end up moving into a protocol, so different document types can specify
// whether or not they implement the protocol. For now we'll just deal with HTML.
// These methods are still in flux; don't rely on them yet.
- (BOOL)canMarkAllTextMatches;
- (WebNSUInteger)markAllMatchesForText:(NSString *)string caseSensitive:(BOOL)caseFlag highlight:(BOOL)highlight limit:(WebNSUInteger)limit;
- (void)unmarkAllTextMatches;
- (NSArray *)rectsForTextMatches;

// Support for disabling registration with the undo manager. This is equivalent to the methods with the same names on NSTextView.
- (BOOL)allowsUndo;
- (void)setAllowsUndo:(BOOL)flag;

- (void)setPageSizeMultiplier:(float)multiplier;

- (float)pageSizeMultiplier;

// Commands for doing page zoom.  Will end up in WebView (WebIBActions) <NSUserInterfaceValidations>
- (BOOL)canZoomPageIn;
- (IBAction)zoomPageIn:(id)sender;
- (BOOL)canZoomPageOut;
- (IBAction)zoomPageOut:(id)sender;
- (BOOL)canResetPageZoom;
- (IBAction)resetPageZoom:(id)sender;

// Sets a master volume control for all media elements in the WebView. Valid values are 0..1.
- (void)setMediaVolume:(float)volume;
- (float)mediaVolume;

// Add visited links
- (void)addVisitedLinks:(NSArray *)visitedLinks;

@end

@interface WebView (WebPrivate)

- (WebInspector *)inspector;

- (void)setBackgroundColor:(NSColor *)backgroundColor;

- (NSColor *)backgroundColor;

- (void)_loadBackForwardListFromOtherView:(WebView *)otherView;


- (void)_dispatchPendingLoadRequests;

+ (NSArray *)_supportedFileExtensions;

+ (BOOL)canShowFile:(NSString *)path;

+ (NSString *)suggestedFileExtensionForMIMEType: (NSString *)MIMEType;

+ (NSString *)_standardUserAgentWithApplicationName:(NSString *)applicationName;

+ (BOOL)canCloseAllWebViews;

// May well become public
- (void)_setFormDelegate:(id<WebFormDelegate>)delegate;
- (id<WebFormDelegate>)_formDelegate;

- (BOOL)_isClosed;

// _close is now replaced by public method -close. It remains here only for backward compatibility
// until callers can be weaned off of it.
- (void)_close;

// Indicates if the WebView is in the midst of a user gesture.
- (BOOL)_isProcessingUserGesture;

// SPI for DumpRenderTree
- (void)_updateActiveState;

+ (void)_registerViewClass:(Class)viewClass representationClass:(Class)representationClass forURLScheme:(NSString *)URLScheme;

+ (void)_unregisterViewClassAndRepresentationClassForMIMEType:(NSString *)MIMEType;

+ (BOOL)_canHandleRequest:(NSURLRequest *)request;

+ (NSString *)_decodeData:(NSData *)data;

+ (void)_setAlwaysUsesComplexTextCodePath:(BOOL)f;
// This is the old name of the above method. Needed for Safari versions that call it.
+ (void)_setAlwaysUseATSU:(BOOL)f;

- (NSCachedURLResponse *)_cachedResponseForURL:(NSURL *)URL;

#if ENABLE_DASHBOARD_SUPPORT
- (void)_addScrollerDashboardRegions:(NSMutableDictionary *)regions;
- (NSDictionary *)_dashboardRegions;

- (void)_setDashboardBehavior:(WebDashboardBehavior)behavior to:(BOOL)flag;
- (BOOL)_dashboardBehavior:(WebDashboardBehavior)behavior;
#endif

+ (void)_setShouldUseFontSmoothing:(BOOL)f;
+ (BOOL)_shouldUseFontSmoothing;

- (void)_setCatchesDelegateExceptions:(BOOL)f;
- (BOOL)_catchesDelegateExceptions;

// These two methods are useful for a test harness that needs a consistent appearance for the focus rings
// regardless of OS X version.
+ (void)_setUsesTestModeFocusRingColor:(BOOL)f;
+ (BOOL)_usesTestModeFocusRingColor;

- (void)setAlwaysShowVerticalScroller:(BOOL)flag;

- (BOOL)alwaysShowVerticalScroller;

- (void)setAlwaysShowHorizontalScroller:(BOOL)flag;

- (BOOL)alwaysShowHorizontalScroller;

- (void)setProhibitsMainFrameScrolling:(BOOL)prohibits;

- (void)_setAdditionalWebPlugInPaths:(NSArray *)newPaths;

- (void)_setInViewSourceMode:(BOOL)flag;

- (BOOL)_inViewSourceMode;

- (void)_attachScriptDebuggerToAllFrames;

- (void)_detachScriptDebuggerFromAllFrames;

- (BOOL)defersCallbacks; // called by QuickTime plug-in
- (void)setDefersCallbacks:(BOOL)defer; // called by QuickTime plug-in

- (BOOL)usesPageCache;
- (void)setUsesPageCache:(BOOL)usesPageCache;

- (WebHistoryItem *)_globalHistoryItem;

- (WebTextIterator *)textIteratorForRect:(NSRect)rect;

#if ENABLE_DASHBOARD_SUPPORT
// <rdar://problem/5217124> Clients other than Dashboard, don't use this.
// As of this writing, Dashboard uses this on Tiger, but not on Leopard or newer.
- (void)handleAuthenticationForResource:(id)identifier challenge:(NSURLAuthenticationChallenge *)challenge fromDataSource:(WebDataSource *)dataSource;
#endif

- (void)_clearUndoRedoOperations;

/* Used to do fast (lower quality) scaling of images so that window resize can be quick. */
- (BOOL)_inFastImageScalingMode;
- (void)_setUseFastImageScalingMode:(BOOL)flag;

- (BOOL)_cookieEnabled;
- (void)_setCookieEnabled:(BOOL)enable;

// SPI for DumpRenderTree
- (void)_executeCoreCommandByName:(NSString *)name value:(NSString *)value;
- (void)_clearMainFrameName;

- (void)_setCustomHTMLTokenizerTimeDelay:(double)timeDelay;
- (void)_setCustomHTMLTokenizerChunkSize:(int)chunkSize;

- (id)_initWithFrame:(NSRect)f frameName:(NSString *)frameName groupName:(NSString *)groupName usesDocumentViews:(BOOL)usesDocumentViews;
- (BOOL)_usesDocumentViews;

- (void)setSelectTrailingWhitespaceEnabled:(BOOL)flag;
- (BOOL)isSelectTrailingWhitespaceEnabled;

- (void)setMemoryCacheDelegateCallsEnabled:(BOOL)suspend;
- (BOOL)areMemoryCacheDelegateCallsEnabled;

- (void)_setJavaScriptURLsAreAllowed:(BOOL)setJavaScriptURLsAreAllowed;

+ (NSCursor *)_pointingHandCursor;

// SPI for DumpRenderTree
- (BOOL)_postsAcceleratedCompositingNotifications;
- (void)_setPostsAcceleratedCompositingNotifications:(BOOL)flag;
- (BOOL)_isUsingAcceleratedCompositing;

// SPI for PluginHalter
+ (BOOL)_isNodeHaltedPlugin:(DOMNode *)node;
+ (BOOL)_hasPluginForNodeBeenHalted:(DOMNode *)node;
+ (void)_restartHaltedPluginForNode:(DOMNode *)node;

// Which pasteboard text is coming from in editing delegate methods such as shouldInsertNode.
- (NSPasteboard *)_insertionPasteboard;

// Whitelists access from an origin (sourceOrigin) to a set of one or more origins described by the parameters:
// - destinationProtocol: The protocol to grant access to.
// - destinationHost: The host to grant access to.
// - allowDestinationSubdomains: If host is a domain, setting this to YES will whitelist host and all its subdomains, recursively.
+ (void)_whiteListAccessFromOrigin:(NSString *)sourceOrigin destinationProtocol:(NSString *)destinationProtocol destinationHost:(NSString *)destinationHost allowDestinationSubdomains:(BOOL)allowDestinationSubdomains;

// Removes all white list entries created with _whiteListAccessFromOrigin.
+ (void)_resetOriginAccessWhiteLists;

+ (void)_addUserScriptToGroup:(NSString *)groupName world:(WebScriptWorld *)world source:(NSString *)source url:(NSURL *)url whitelist:(NSArray *)whitelist blacklist:(NSArray *)blacklist injectionTime:(WebUserScriptInjectionTime)injectionTime;
+ (void)_addUserStyleSheetToGroup:(NSString *)groupName world:(WebScriptWorld *)world source:(NSString *)source url:(NSURL *)url whitelist:(NSArray *)whitelist blacklist:(NSArray *)blacklist;
+ (void)_removeUserScriptFromGroup:(NSString *)groupName world:(WebScriptWorld *)world url:(NSURL *)url;
+ (void)_removeUserStyleSheetFromGroup:(NSString *)groupName world:(WebScriptWorld *)world url:(NSURL *)url;
+ (void)_removeUserScriptsFromGroup:(NSString *)groupName world:(WebScriptWorld *)world;
+ (void)_removeUserStyleSheetsFromGroup:(NSString *)groupName world:(WebScriptWorld *)world;
+ (void)_removeAllUserContentFromGroup:(NSString *)groupName;

- (BOOL)cssAnimationsSuspended;

- (void)setCSSAnimationsSuspended:(BOOL)suspended;

+ (void)_setDomainRelaxationForbidden:(BOOL)forbidden forURLScheme:(NSString *)scheme;

@end

@interface WebView (WebViewPrintingPrivate)
- (void)_adjustPrintingMarginsForHeaderAndFooter;

- (void)_drawHeaderAndFooter;
@end

@interface WebView (WebViewGrammarChecking)

// FIXME: These two methods should be merged into WebViewEditing when we're not in API freeze
- (BOOL)isGrammarCheckingEnabled;
#ifndef BUILDING_ON_TIGER
- (void)setGrammarCheckingEnabled:(BOOL)flag;

// FIXME: This method should be merged into WebIBActions when we're not in API freeze
- (void)toggleGrammarChecking:(id)sender;
#endif

@end

@interface WebView (WebViewTextChecking)

- (BOOL)isAutomaticQuoteSubstitutionEnabled;
- (BOOL)isAutomaticLinkDetectionEnabled;
- (BOOL)isAutomaticDashSubstitutionEnabled;
- (BOOL)isAutomaticTextReplacementEnabled;
- (BOOL)isAutomaticSpellingCorrectionEnabled;
#if !defined(BUILDING_ON_TIGER) && !defined(BUILDING_ON_LEOPARD)
- (void)setAutomaticQuoteSubstitutionEnabled:(BOOL)flag;
- (void)toggleAutomaticQuoteSubstitution:(id)sender;
- (void)setAutomaticLinkDetectionEnabled:(BOOL)flag;
- (void)toggleAutomaticLinkDetection:(id)sender;
- (void)setAutomaticDashSubstitutionEnabled:(BOOL)flag;
- (void)toggleAutomaticDashSubstitution:(id)sender;
- (void)setAutomaticTextReplacementEnabled:(BOOL)flag;
- (void)toggleAutomaticTextReplacement:(id)sender;
- (void)setAutomaticSpellingCorrectionEnabled:(BOOL)flag;
- (void)toggleAutomaticSpellingCorrection:(id)sender;
#endif

@end

@interface WebView (WebViewEditingInMail)
- (void)_insertNewlineInQuotedContent;
- (void)_replaceSelectionWithNode:(DOMNode *)node matchStyle:(BOOL)matchStyle;
- (BOOL)_selectionIsCaret;
- (BOOL)_selectionIsAll;
@end

@protocol WebGeolocationProvider <NSObject>
- (void)registerWebView:(WebView *)webView;
- (void)unregisterWebView:(WebView *)webView;
- (WebGeolocationPosition *)lastPosition;
@end

@interface WebView (WebViewGeolocation)
- (void)_setGeolocationProvider:(id<WebGeolocationProvider>)locationProvider;
- (id<WebGeolocationProvider>)_geolocationProvider;

- (void)_geolocationDidChangePosition:(WebGeolocationPosition *)position;
- (void)_geolocationDidFailWithError:(NSError *)error;
@end

@interface NSObject (WebFrameLoadDelegatePrivate)
- (void)webView:(WebView *)sender didFirstLayoutInFrame:(WebFrame *)frame;

// didFinishDocumentLoadForFrame is sent when the document has finished loading, though not necessarily all
// of its subresources.
// FIXME 5259339: Currently this callback is not sent for (some?) pages loaded entirely from the cache.
- (void)webView:(WebView *)sender didFinishDocumentLoadForFrame:(WebFrame *)frame;

// Addresses 4192534.  SPI for now.
- (void)webView:(WebView *)sender didHandleOnloadEventsForFrame:(WebFrame *)frame;

- (void)webView:(WebView *)sender didFirstVisuallyNonEmptyLayoutInFrame:(WebFrame *)frame;

// For implementing the WebInspector's test harness
- (void)webView:(WebView *)webView didClearInspectorWindowObject:(WebScriptObject *)windowObject forFrame:(WebFrame *)frame;

@end

@interface NSObject (WebResourceLoadDelegatePrivate)
// Addresses <rdar://problem/5008925> - SPI for now
- (NSCachedURLResponse *)webView:(WebView *)sender resource:(id)identifier willCacheResponse:(NSCachedURLResponse *)response fromDataSource:(WebDataSource *)dataSource;
@end

#undef WebNSInteger
#undef WebNSUInteger
