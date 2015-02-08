

#import <Cocoa/Cocoa.h>

@class NSError;
@class NSURLResponse;
@class NSURLRequest;
@class WebView;
@class WebFrame;
@class WebPolicyPrivate;


typedef enum {
    WebNavigationTypeLinkClicked,
    WebNavigationTypeFormSubmitted,
    WebNavigationTypeBackForward,
    WebNavigationTypeReload,
    WebNavigationTypeFormResubmitted,
    WebNavigationTypeOther
} WebNavigationType;

extern NSString *WebActionNavigationTypeKey; // NSNumber (WebNavigationType)
extern NSString *WebActionElementKey; // NSDictionary of element info
extern NSString *WebActionButtonKey; // NSNumber (0 for left button, 1 for middle button, 2 for right button)
extern NSString *WebActionModifierFlagsKey; // NSNumber (unsigned)
extern NSString *WebActionOriginalURLKey; // NSURL


@protocol WebPolicyDecisionListener <NSObject>

- (void)use;
- (void)download;

- (void)ignore;

@end


@interface NSObject (WebPolicyDelegate)

- (void)webView:(WebView *)webView decidePolicyForNavigationAction:(NSDictionary *)actionInformation
                                                           request:(NSURLRequest *)request
                                                             frame:(WebFrame *)frame
                                                  decisionListener:(id<WebPolicyDecisionListener>)listener;

- (void)webView:(WebView *)webView decidePolicyForNewWindowAction:(NSDictionary *)actionInformation
                                                          request:(NSURLRequest *)request
                                                     newFrameName:(NSString *)frameName
                                                 decisionListener:(id<WebPolicyDecisionListener>)listener;

- (void)webView:(WebView *)webView decidePolicyForMIMEType:(NSString *)type
                                                   request:(NSURLRequest *)request
                                                     frame:(WebFrame *)frame
                                          decisionListener:(id<WebPolicyDecisionListener>)listener;

- (void)webView:(WebView *)webView unableToImplementPolicyWithError:(NSError *)error frame:(WebFrame *)frame;

@end
