

#import <Cocoa/Cocoa.h>
#import <Foundation/NSURLRequest.h>
#import <JavaScriptCore/WebKitAvailability.h>

#if MAC_OS_X_VERSION_MAX_ALLOWED <= MAC_OS_X_VERSION_10_4
#define WebNSUInteger unsigned int
#else
#define WebNSUInteger NSUInteger
#endif


enum {
    WebMenuItemTagOpenLinkInNewWindow=1,
    WebMenuItemTagDownloadLinkToDisk,
    WebMenuItemTagCopyLinkToClipboard,
    WebMenuItemTagOpenImageInNewWindow,
    WebMenuItemTagDownloadImageToDisk,
    WebMenuItemTagCopyImageToClipboard,
    WebMenuItemTagOpenFrameInNewWindow,
    WebMenuItemTagCopy,
    WebMenuItemTagGoBack,
    WebMenuItemTagGoForward,
    WebMenuItemTagStop,
    WebMenuItemTagReload,
    WebMenuItemTagCut,
    WebMenuItemTagPaste,
    WebMenuItemTagSpellingGuess,
    WebMenuItemTagNoGuessesFound,
    WebMenuItemTagIgnoreSpelling,
    WebMenuItemTagLearnSpelling,
    WebMenuItemTagOther,
    WebMenuItemTagSearchInSpotlight,
    WebMenuItemTagSearchWeb,
    WebMenuItemTagLookUpInDictionary,
    WebMenuItemTagOpenWithDefaultApplication,
    WebMenuItemPDFActualSize,
    WebMenuItemPDFZoomIn,
    WebMenuItemPDFZoomOut,
    WebMenuItemPDFAutoSize,
    WebMenuItemPDFSinglePage,
    WebMenuItemPDFFacingPages,
    WebMenuItemPDFContinuous,
    WebMenuItemPDFNextPage,
    WebMenuItemPDFPreviousPage,
};

typedef enum {
    WebDragDestinationActionNone    = 0,
    WebDragDestinationActionDHTML   = 1,
    WebDragDestinationActionEdit    = 2,
    WebDragDestinationActionLoad    = 4,
    WebDragDestinationActionAny     = UINT_MAX
} WebDragDestinationAction;

typedef enum {
    WebDragSourceActionNone         = 0,
    WebDragSourceActionDHTML        = 1,
    WebDragSourceActionImage        = 2,
    WebDragSourceActionLink         = 4,
    WebDragSourceActionSelection    = 8,
    WebDragSourceActionAny          = UINT_MAX
} WebDragSourceAction;

@protocol WebOpenPanelResultListener <NSObject>

- (void)chooseFilename:(NSString *)fileName;

- (void)chooseFilenames:(NSArray *)fileNames WEBKIT_OBJC_METHOD_ANNOTATION(AVAILABLE_IN_WEBKIT_VERSION_4_0);

- (void)cancel;

@end

@class WebView;

@interface NSObject (WebUIDelegate)

- (WebView *)webView:(WebView *)sender createWebViewWithRequest:(NSURLRequest *)request;

- (void)webViewShow:(WebView *)sender;

- (WebView *)webView:(WebView *)sender createWebViewModalDialogWithRequest:(NSURLRequest *)request;

- (void)webViewRunModal:(WebView *)sender;

- (void)webViewClose:(WebView *)sender;

- (void)webViewFocus:(WebView *)sender;

- (void)webViewUnfocus:(WebView *)sender;

- (NSResponder *)webViewFirstResponder:(WebView *)sender;

- (void)webView:(WebView *)sender makeFirstResponder:(NSResponder *)responder;

- (void)webView:(WebView *)sender setStatusText:(NSString *)text;

- (NSString *)webViewStatusText:(WebView *)sender;

- (BOOL)webViewAreToolbarsVisible:(WebView *)sender;

- (void)webView:(WebView *)sender setToolbarsVisible:(BOOL)visible;

- (BOOL)webViewIsStatusBarVisible:(WebView *)sender;

- (void)webView:(WebView *)sender setStatusBarVisible:(BOOL)visible;

- (BOOL)webViewIsResizable:(WebView *)sender;

- (void)webView:(WebView *)sender setResizable:(BOOL)resizable;

- (void)webView:(WebView *)sender setFrame:(NSRect)frame;

- (NSRect)webViewFrame:(WebView *)sender;

- (void)webView:(WebView *)sender runJavaScriptAlertPanelWithMessage:(NSString *)message initiatedByFrame:(WebFrame *)frame;

- (BOOL)webView:(WebView *)sender runJavaScriptConfirmPanelWithMessage:(NSString *)message initiatedByFrame:(WebFrame *)frame;

- (NSString *)webView:(WebView *)sender runJavaScriptTextInputPanelWithPrompt:(NSString *)prompt defaultText:(NSString *)defaultText initiatedByFrame:(WebFrame *)frame;

- (BOOL)webView:(WebView *)sender runBeforeUnloadConfirmPanelWithMessage:(NSString *)message initiatedByFrame:(WebFrame *)frame;

- (void)webView:(WebView *)sender runOpenPanelForFileButtonWithResultListener:(id<WebOpenPanelResultListener>)resultListener;

- (void)webView:(WebView *)sender runOpenPanelForFileButtonWithResultListener:(id<WebOpenPanelResultListener>)resultListener allowMultipleFiles:(BOOL)allowMultipleFiles WEBKIT_OBJC_METHOD_ANNOTATION(AVAILABLE_IN_WEBKIT_VERSION_4_0);

- (void)webView:(WebView *)sender mouseDidMoveOverElement:(NSDictionary *)elementInformation modifierFlags:(WebNSUInteger)modifierFlags;

- (NSArray *)webView:(WebView *)sender contextMenuItemsForElement:(NSDictionary *)element defaultMenuItems:(NSArray *)defaultMenuItems;

- (BOOL)webView:(WebView *)webView validateUserInterfaceItem:(id <NSValidatedUserInterfaceItem>)item defaultValidation:(BOOL)defaultValidation;

- (BOOL)webView:(WebView *)webView shouldPerformAction:(SEL)action fromSender:(id)sender;

- (WebNSUInteger)webView:(WebView *)webView dragDestinationActionMaskForDraggingInfo:(id <NSDraggingInfo>)draggingInfo;

- (void)webView:(WebView *)webView willPerformDragDestinationAction:(WebDragDestinationAction)action forDraggingInfo:(id <NSDraggingInfo>)draggingInfo;

- (WebNSUInteger)webView:(WebView *)webView dragSourceActionMaskForPoint:(NSPoint)point;

- (void)webView:(WebView *)webView willPerformDragSourceAction:(WebDragSourceAction)action fromPoint:(NSPoint)point withPasteboard:(NSPasteboard *)pasteboard;

- (void)webView:(WebView *)sender printFrameView:(WebFrameView *)frameView;

- (float)webViewHeaderHeight:(WebView *)sender;

- (float)webViewFooterHeight:(WebView *)sender;

- (void)webView:(WebView *)sender drawHeaderInRect:(NSRect)rect;

- (void)webView:(WebView *)sender drawFooterInRect:(NSRect)rect;

// The following delegate methods are deprecated in favor of the ones above that specify
// the WebFrame whose JavaScript initiated this call.
- (void)webView:(WebView *)sender runJavaScriptAlertPanelWithMessage:(NSString *)message WEBKIT_OBJC_METHOD_ANNOTATION(AVAILABLE_WEBKIT_VERSION_1_0_AND_LATER_BUT_DEPRECATED_IN_WEBKIT_VERSION_3_0);
- (BOOL)webView:(WebView *)sender runJavaScriptConfirmPanelWithMessage:(NSString *)message WEBKIT_OBJC_METHOD_ANNOTATION(AVAILABLE_WEBKIT_VERSION_1_0_AND_LATER_BUT_DEPRECATED_IN_WEBKIT_VERSION_3_0);
- (NSString *)webView:(WebView *)sender runJavaScriptTextInputPanelWithPrompt:(NSString *)prompt defaultText:(NSString *)defaultText WEBKIT_OBJC_METHOD_ANNOTATION(AVAILABLE_WEBKIT_VERSION_1_0_AND_LATER_BUT_DEPRECATED_IN_WEBKIT_VERSION_3_0);

// The following delegate methods are deprecated. Content rect calculations are now done automatically.
- (void)webView:(WebView *)sender setContentRect:(NSRect)frame WEBKIT_OBJC_METHOD_ANNOTATION(AVAILABLE_WEBKIT_VERSION_1_0_AND_LATER_BUT_DEPRECATED_IN_WEBKIT_VERSION_3_0);
- (NSRect)webViewContentRect:(WebView *)sender WEBKIT_OBJC_METHOD_ANNOTATION(AVAILABLE_WEBKIT_VERSION_1_0_AND_LATER_BUT_DEPRECATED_IN_WEBKIT_VERSION_3_0);

@end

#undef WebNSUInteger
