

#import <Cocoa/Cocoa.h>
#import <WebKit/WebView.h>

@interface NavigationController : NSObject
{
    enum { None, Load, GoBack, ExecuteScript } pendingAction;
    NSString *pendingScript;
    NSURLRequest *pendingRequest;
}
- (void)webView:(WebView *)sender didFinishLoadForFrame:(WebFrame *)frame;
@end
