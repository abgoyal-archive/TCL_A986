

// This header contains WebView declarations that can be used anywhere in WebKit, but are neither SPI nor API.

#import "WebTypesInternal.h"

@class WebView;

struct WebResourceDelegateImplementationCache {
    IMP didCancelAuthenticationChallengeFunc;
    IMP didReceiveAuthenticationChallengeFunc;
    IMP identifierForRequestFunc;
    IMP willSendRequestFunc;
    IMP didReceiveResponseFunc;
    IMP didReceiveContentLengthFunc;
    IMP didFinishLoadingFromDataSourceFunc;
    IMP didFailLoadingWithErrorFromDataSourceFunc;
    IMP didLoadResourceFromMemoryCacheFunc;
    IMP willCacheResponseFunc;
    IMP plugInFailedWithErrorFunc;
    IMP shouldUseCredentialStorageFunc;
};

struct WebFrameLoadDelegateImplementationCache {
    IMP didClearWindowObjectForFrameFunc;
    IMP didClearWindowObjectForFrameInScriptWorldFunc;
    IMP didClearInspectorWindowObjectForFrameFunc;
    IMP windowScriptObjectAvailableFunc;
    IMP didHandleOnloadEventsForFrameFunc;
    IMP didReceiveServerRedirectForProvisionalLoadForFrameFunc;
    IMP didCancelClientRedirectForFrameFunc;
    IMP willPerformClientRedirectToURLDelayFireDateForFrameFunc;
    IMP didChangeLocationWithinPageForFrameFunc;
    IMP didPushStateWithinPageForFrameFunc;
    IMP didReplaceStateWithinPageForFrameFunc;
    IMP didPopStateWithinPageForFrameFunc;
    IMP willCloseFrameFunc;
    IMP didStartProvisionalLoadForFrameFunc;
    IMP didReceiveTitleForFrameFunc;
    IMP didCommitLoadForFrameFunc;
    IMP didFailProvisionalLoadWithErrorForFrameFunc;
    IMP didFailLoadWithErrorForFrameFunc;
    IMP didFinishLoadForFrameFunc;
    IMP didFirstLayoutInFrameFunc;
    IMP didFirstVisuallyNonEmptyLayoutInFrameFunc;
    IMP didReceiveIconForFrameFunc;
    IMP didFinishDocumentLoadForFrameFunc;
    IMP didDisplayInsecureContentFunc;
    IMP didRunInsecureContentFunc;
};

struct WebScriptDebugDelegateImplementationCache {
    BOOL didParseSourceExpectsBaseLineNumber;
    IMP didParseSourceFunc;
    IMP failedToParseSourceFunc;
    IMP didEnterCallFrameFunc;
    IMP willExecuteStatementFunc;
    IMP willLeaveCallFrameFunc;
    IMP exceptionWasRaisedFunc;
};

struct WebHistoryDelegateImplementationCache {
    IMP navigatedFunc;
    IMP clientRedirectFunc;
    IMP serverRedirectFunc;
    IMP setTitleFunc;
    IMP populateVisitedLinksFunc;
};

WebResourceDelegateImplementationCache* WebViewGetResourceLoadDelegateImplementations(WebView *);
WebFrameLoadDelegateImplementationCache* WebViewGetFrameLoadDelegateImplementations(WebView *);
WebScriptDebugDelegateImplementationCache* WebViewGetScriptDebugDelegateImplementations(WebView *);
WebHistoryDelegateImplementationCache* WebViewGetHistoryDelegateImplementations(WebView *webView);

id CallFormDelegate(WebView *, SEL, id, id);
id CallFormDelegate(WebView *self, SEL selector, id object1, id object2, id object3, id object4, id object5);
BOOL CallFormDelegateReturningBoolean(BOOL, WebView *, SEL, id, SEL, id);

id CallUIDelegate(WebView *, SEL);
id CallUIDelegate(WebView *, SEL, id);
id CallUIDelegate(WebView *, SEL, NSRect);
id CallUIDelegate(WebView *, SEL, id, id);
id CallUIDelegate(WebView *, SEL, id, BOOL);
id CallUIDelegate(WebView *, SEL, id, id, id);
id CallUIDelegate(WebView *, SEL, id, NSUInteger);
float CallUIDelegateReturningFloat(WebView *, SEL);
BOOL CallUIDelegateReturningBoolean(BOOL, WebView *, SEL);
BOOL CallUIDelegateReturningBoolean(BOOL, WebView *, SEL, id);
BOOL CallUIDelegateReturningBoolean(BOOL, WebView *, SEL, id, id);
BOOL CallUIDelegateReturningBoolean(BOOL, WebView *, SEL, id, BOOL);
BOOL CallUIDelegateReturningBoolean(BOOL, WebView *, SEL, id, BOOL, id);

id CallFrameLoadDelegate(IMP, WebView *, SEL);
id CallFrameLoadDelegate(IMP, WebView *, SEL, id);
id CallFrameLoadDelegate(IMP, WebView *, SEL, id, id);
id CallFrameLoadDelegate(IMP, WebView *, SEL, id, id, id);
id CallFrameLoadDelegate(IMP, WebView *, SEL, id, id, id, id);
id CallFrameLoadDelegate(IMP, WebView *, SEL, id, NSTimeInterval, id, id);

id CallResourceLoadDelegate(IMP, WebView *, SEL, id, id);
id CallResourceLoadDelegate(IMP, WebView *, SEL, id, id, id);
id CallResourceLoadDelegate(IMP, WebView *, SEL, id, id, id, id);
id CallResourceLoadDelegate(IMP, WebView *, SEL, id, NSInteger, id);
id CallResourceLoadDelegate(IMP, WebView *, SEL, id, id, NSInteger, id);

BOOL CallResourceLoadDelegateReturningBoolean(BOOL, IMP, WebView *, SEL, id, id);

id CallScriptDebugDelegate(IMP, WebView *, SEL, id, id, NSInteger, id);
id CallScriptDebugDelegate(IMP, WebView *, SEL, id, NSInteger, id, NSInteger, id);
id CallScriptDebugDelegate(IMP, WebView *, SEL, id, NSInteger, id, id, id);
id CallScriptDebugDelegate(IMP, WebView *, SEL, id, NSInteger, NSInteger, id);

id CallHistoryDelegate(IMP, WebView *, SEL);
id CallHistoryDelegate(IMP, WebView *, SEL, id, id);
id CallHistoryDelegate(IMP, WebView *, SEL, id, id, id);
