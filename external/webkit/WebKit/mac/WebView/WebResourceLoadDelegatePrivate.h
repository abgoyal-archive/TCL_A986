

#if MAC_OS_X_VERSION_MAX_ALLOWED <= MAC_OS_X_VERSION_10_4
#define WebNSInteger int
#else
#define WebNSInteger NSInteger
#endif

@class WebView;
@class WebDataSource;
@class NSURLAuthenticationChallenge;
@class NSURLResponse;
@class NSURLRequest;

@interface NSObject (WebResourceLoadDelegatePrivate)

- (void)webView:(WebView *)webView didLoadResourceFromMemoryCache:(NSURLRequest *)request response:(NSURLResponse *)response length:(WebNSInteger)length fromDataSource:(WebDataSource *)dataSource;
- (BOOL)webView:(WebView *)webView resource:(id)identifier shouldUseCredentialStorageForDataSource:(WebDataSource *)dataSource;

@end

#undef WebNSInteger
