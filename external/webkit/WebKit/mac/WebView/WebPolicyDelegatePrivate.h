

#import <WebKit/WebPolicyDelegate.h>

@class WebHistoryItem;
@class WebPolicyDecisionListenerPrivate;

extern NSString *WebActionFormKey; // HTMLFormElement

typedef enum {
    WebNavigationTypePlugInRequest = WebNavigationTypeOther + 1
} WebExtraNavigationType;

@interface WebPolicyDecisionListener : NSObject <WebPolicyDecisionListener>
{
@private
    WebPolicyDecisionListenerPrivate *_private;
}
- (id)_initWithTarget:(id)target action:(SEL)action;
- (void)_invalidate;
@end

@interface NSObject (WebPolicyDelegatePrivate)
// Needed for <rdar://problem/3951283> can view pages from the back/forward cache that should be disallowed by Parental Controls
- (BOOL)webView:(WebView *)webView shouldGoToHistoryItem:(WebHistoryItem *)item;
@end
