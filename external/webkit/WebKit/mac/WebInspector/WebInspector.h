

#import <Foundation/NSObject.h>

@class WebView;

@interface WebInspector : NSObject
{
    WebView *_webView;
}
- (id)initWithWebView:(WebView *)webView;
- (void)webViewClosed;
- (void)show:(id)sender;
- (void)showConsole:(id)sender;
- (void)close:(id)sender;
- (void)attach:(id)sender;
- (void)detach:(id)sender;

- (BOOL)isDebuggingJavaScript;
- (void)toggleDebuggingJavaScript:(id)sender;
- (void)startDebuggingJavaScript:(id)sender;
- (void)stopDebuggingJavaScript:(id)sender;

- (BOOL)isJavaScriptProfilingEnabled;
- (void)setJavaScriptProfilingEnabled:(BOOL)enabled;
- (BOOL)isTimelineProfilingEnabled;
- (void)setTimelineProfilingEnabled:(BOOL)enabled;

- (BOOL)isProfilingJavaScript;
- (void)toggleProfilingJavaScript:(id)sender;
- (void)startProfilingJavaScript:(id)sender;
- (void)stopProfilingJavaScript:(id)sender;
@end
