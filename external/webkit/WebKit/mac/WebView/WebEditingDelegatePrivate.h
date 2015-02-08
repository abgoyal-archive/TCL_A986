

#import <WebKit/WebEditingDelegate.h>

@class DOMHTMLElement;

@interface NSObject (WebViewEditingDelegatePrivate)
- (BOOL)webView:(WebView *)webView shouldShowDeleteInterfaceForElement:(DOMHTMLElement *)element;
- (void)webView:(WebView *)webView didWriteSelectionToPasteboard:(NSPasteboard *)pasteboard;
- (void)webView:(WebView *)webView didSetSelectionTypesForPasteboard:(NSPasteboard *)pasteboard;
- (BOOL)webView:(WebView *)webView shouldMoveRangeAfterDelete:(DOMRange *)range replacingRange:(DOMRange *)rangeToBeReplaced;
@end
