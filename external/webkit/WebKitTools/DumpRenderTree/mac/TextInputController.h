

#import <Foundation/Foundation.h>

@class WebView;
@class WebHTMLView;
@class WebScriptObject;

@interface TextInputController : NSObject
{
    WebView *webView;
    WebHTMLView *inputMethodView;
    WebScriptObject *inputMethodHandler;    
}
- (id)initWithWebView:(WebView *)view;
@end
