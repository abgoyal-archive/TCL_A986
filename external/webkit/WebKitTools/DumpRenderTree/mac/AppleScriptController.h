

#import <Foundation/Foundation.h>

@class WebView;

@interface AppleScriptController : NSObject
{
    WebView *webView;
}
- (id)initWithWebView:(WebView *)view;
@end
