

#import <WebKit/WebFrameLoadDelegate.h>

@class WebScriptWorld;
@class WebSecurityOrigin;

@interface NSObject (WebFrameLoadDelegatePrivate)

- (void)webViewDidDisplayInsecureContent:(WebView *)webView;

- (void)webView:(WebView *)webView didRunInsecureContent:(WebSecurityOrigin *)origin;

- (void)webView:(WebView *)webView didClearWindowObjectForFrame:(WebFrame *)frame inScriptWorld:(WebScriptWorld *)world;

- (void)webView:(WebView *)webView didPushStateWithinPageForFrame:(WebFrame *)frame;
- (void)webView:(WebView *)webView didReplaceStateWithinPageForFrame:(WebFrame *)frame;
- (void)webView:(WebView *)webView didPopStateWithinPageForFrame:(WebFrame *)frame;

@end
