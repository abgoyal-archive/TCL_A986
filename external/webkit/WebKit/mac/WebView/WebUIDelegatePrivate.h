

#import <WebKit/WebUIDelegate.h>

#if !defined(ENABLE_DASHBOARD_SUPPORT)
#define ENABLE_DASHBOARD_SUPPORT 1
#endif

// Mail on Tiger expects the old value for WebMenuItemTagSearchInGoogle
#define WebMenuItemTagSearchInGoogle OldWebMenuItemTagSearchWeb

#define WEBMENUITEMTAG_WEBKIT_3_0_SPI_START 2000
enum { 
    // The next three values were used in WebKit 2.0 for SPI. In WebKit 3.0 these are API, with different values.
    OldWebMenuItemTagSearchInSpotlight = 1000,
    OldWebMenuItemTagSearchWeb,
    OldWebMenuItemTagLookUpInDictionary,
    // FIXME: These should move to WebUIDelegate.h as part of the WebMenuItemTag enum there, when we're not in API freeze 
    // Note that these values must be kept aligned with values in WebCore/ContextMenuItem.h
    WebMenuItemTagOpenLink = WEBMENUITEMTAG_WEBKIT_3_0_SPI_START,
    WebMenuItemTagIgnoreGrammar,
    WebMenuItemTagSpellingMenu,
    WebMenuItemTagShowSpellingPanel,
    WebMenuItemTagCheckSpelling,
    WebMenuItemTagCheckSpellingWhileTyping,
    WebMenuItemTagCheckGrammarWithSpelling,
    WebMenuItemTagFontMenu,
    WebMenuItemTagShowFonts,
    WebMenuItemTagBold,
    WebMenuItemTagItalic,
    WebMenuItemTagUnderline,
    WebMenuItemTagOutline,
    WebMenuItemTagStyles,
    WebMenuItemTagShowColors,
    WebMenuItemTagSpeechMenu,
    WebMenuItemTagStartSpeaking,
    WebMenuItemTagStopSpeaking,
    WebMenuItemTagWritingDirectionMenu,
    WebMenuItemTagDefaultDirection,
    WebMenuItemTagLeftToRight,
    WebMenuItemTagRightToLeft,
    WebMenuItemPDFSinglePageScrolling,
    WebMenuItemPDFFacingPagesScrolling,
    WebMenuItemTagInspectElement,
    WebMenuItemTagTextDirectionMenu,
    WebMenuItemTagTextDirectionDefault,
    WebMenuItemTagTextDirectionLeftToRight,
    WebMenuItemTagTextDirectionRightToLeft,
    WebMenuItemTagCorrectSpellingAutomatically,
    WebMenuItemTagSubstitutionsMenu,
    WebMenuItemTagShowSubstitutions,
    WebMenuItemTagSmartCopyPaste,
    WebMenuItemTagSmartQuotes,
    WebMenuItemTagSmartDashes,
    WebMenuItemTagSmartLinks,
    WebMenuItemTagTextReplacement,
    WebMenuItemTagTransformationsMenu,
    WebMenuItemTagMakeUpperCase,
    WebMenuItemTagMakeLowerCase,
    WebMenuItemTagCapitalize,
    WebMenuItemTagChangeBack,
    WebMenuItemTagBaseApplication = 10000
};

@class WebSecurityOrigin;

@protocol WebGeolocationPolicyListener <NSObject>
- (void)allow;
- (void)deny;
@end

@interface NSObject (WebUIDelegatePrivate)

- (void)webView:(WebView *)webView addMessageToConsole:(NSDictionary *)message;

- (NSView *)webView:(WebView *)webView plugInViewWithArguments:(NSDictionary *)arguments;

#if ENABLE_DASHBOARD_SUPPORT
// regions is an dictionary whose keys are regions label and values are arrays of WebDashboardRegions.
- (void)webView:(WebView *)webView dashboardRegionsChanged:(NSDictionary *)regions;
#endif

- (void)webView:(WebView *)sender dragImage:(NSImage *)anImage at:(NSPoint)viewLocation offset:(NSSize)initialOffset event:(NSEvent *)event pasteboard:(NSPasteboard *)pboard source:(id)sourceObj slideBack:(BOOL)slideFlag forView:(NSView *)view;
- (void)webView:(WebView *)sender didDrawRect:(NSRect)rect;
- (void)webView:(WebView *)sender didScrollDocumentInFrameView:(WebFrameView *)frameView;
// FIXME: If we ever make this method public, it should include a WebFrame parameter.
- (BOOL)webViewShouldInterruptJavaScript:(WebView *)sender;
- (void)webView:(WebView *)sender willPopupMenu:(NSMenu *)menu;
- (void)webView:(WebView *)sender contextMenuItemSelected:(NSMenuItem *)item forElement:(NSDictionary *)element;
- (void)webView:(WebView *)sender saveFrameView:(WebFrameView *)frameView showingPanel:(BOOL)showingPanel;
- (BOOL)webView:(WebView *)sender shouldHaltPlugin:(DOMNode *)pluginNode isWindowed:(BOOL)isWindowed pluginName:(NSString *)pluginName;
- (void)webView:(WebView *)sender frame:(WebFrame *)frame exceededDatabaseQuotaForSecurityOrigin:(WebSecurityOrigin *)origin database:(NSString *)databaseIdentifier;

- (WebView *)webView:(WebView *)sender createWebViewWithRequest:(NSURLRequest *)request windowFeatures:(NSDictionary *)features;

- (BOOL)webView:(WebView *)sender shouldReplaceUploadFile:(NSString *)path usingGeneratedFilename:(NSString **)filename;
- (NSString *)webView:(WebView *)sender generateReplacementFile:(NSString *)path;

- (void)webView:(WebView *)webView decidePolicyForGeolocationRequestFromOrigin:(WebSecurityOrigin *)origin
                                                                         frame:(WebFrame *)frame
                                                                      listener:(id<WebGeolocationPolicyListener>)listener;

- (void)webView:(WebView *)sender formStateDidChangeForNode:(DOMNode *)node;
- (void)webView:(WebView *)sender formDidFocusNode:(DOMNode *)node;
- (void)webView:(WebView *)sender formDidBlurNode:(DOMNode *)node;

- (void)webView:(WebView *)sender printFrame:(WebFrame *)frame;

@end
