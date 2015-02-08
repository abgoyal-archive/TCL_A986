

#import <Foundation/Foundation.h>

@class WebFrame;
@class WebNavigationData;
@class WebView;

@interface NSObject (WebHistoryDelegate)

- (void)webView:(WebView *)webView didNavigateWithNavigationData:(WebNavigationData *)navigationData inFrame:(WebFrame *)webFrame;
                              
- (void)webView:(WebView *)webView didPerformClientRedirectFromURL:(NSString *)sourceURL toURL:(NSString *)destinationURL inFrame:(WebFrame *)webFrame;

- (void)webView:(WebView *)webView didPerformServerRedirectFromURL:(NSString *)sourceURL toURL:(NSString *)destinationURL inFrame:(WebFrame *)webFrame;

- (void)webView:(WebView *)webView updateHistoryTitle:(NSString *)title forURL:(NSString *)url;

- (void)populateVisitedLinksForWebView:(WebView *)webView;

@end
