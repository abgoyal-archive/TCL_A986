

#import <Cocoa/Cocoa.h>
#import <JavaScriptCore/WebKitAvailability.h>

@class NSError;
@class WebFrame;
@class WebScriptObject;
@class WebView;

@interface NSObject (WebFrameLoadDelegate)

- (void)webView:(WebView *)sender didStartProvisionalLoadForFrame:(WebFrame *)frame;

- (void)webView:(WebView *)sender didReceiveServerRedirectForProvisionalLoadForFrame:(WebFrame *)frame;

- (void)webView:(WebView *)sender didFailProvisionalLoadWithError:(NSError *)error forFrame:(WebFrame *)frame;

- (void)webView:(WebView *)sender didCommitLoadForFrame:(WebFrame *)frame;

- (void)webView:(WebView *)sender didReceiveTitle:(NSString *)title forFrame:(WebFrame *)frame;

- (void)webView:(WebView *)sender didReceiveIcon:(NSImage *)image forFrame:(WebFrame *)frame;

- (void)webView:(WebView *)sender didFinishLoadForFrame:(WebFrame *)frame;

- (void)webView:(WebView *)sender didFailLoadWithError:(NSError *)error forFrame:(WebFrame *)frame;

- (void)webView:(WebView *)sender didChangeLocationWithinPageForFrame:(WebFrame *)frame;

- (void)webView:(WebView *)sender willPerformClientRedirectToURL:(NSURL *)URL delay:(NSTimeInterval)seconds fireDate:(NSDate *)date forFrame:(WebFrame *)frame;

- (void)webView:(WebView *)sender didCancelClientRedirectForFrame:(WebFrame *)frame;

- (void)webView:(WebView *)sender willCloseFrame:(WebFrame *)frame;

- (void)webView:(WebView *)webView didClearWindowObject:(WebScriptObject *)windowObject forFrame:(WebFrame *)frame;

- (void)webView:(WebView *)webView windowScriptObjectAvailable:(WebScriptObject *)windowScriptObject WEBKIT_OBJC_METHOD_ANNOTATION(AVAILABLE_WEBKIT_VERSION_1_3_AND_LATER_BUT_DEPRECATED_IN_WEBKIT_VERSION_3_0);

@end
