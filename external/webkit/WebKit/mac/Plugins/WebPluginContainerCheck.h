

#import <Foundation/Foundation.h>

@class NSURLRequest;
@class NSString;
@class WebFrame;
@class WebView;
@class WebPolicyDecisionListener;

@protocol WebPluginContainerCheckController <NSObject>
- (void)_webPluginContainerCancelCheckIfAllowedToLoadRequest:(id)checkIdentifier;
- (WebFrame *)webFrame;
- (WebView *)webView;
@end

@interface WebPluginContainerCheck : NSObject
{
    NSURLRequest *_request;
    NSString *_target;
    id <WebPluginContainerCheckController> _controller;
    id _resultObject;
    SEL _resultSelector;
    id _contextInfo;
    BOOL _done;
    WebPolicyDecisionListener *_listener;
}

+ (id)checkWithRequest:(NSURLRequest *)request target:(NSString *)target resultObject:(id)obj selector:(SEL)selector controller:(id <WebPluginContainerCheckController>)controller contextInfo:(id)/*optional*/contextInfo; 
- (void)start;
- (void)cancel;
- (id)contextInfo;

@end
