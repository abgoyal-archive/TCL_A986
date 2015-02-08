

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

@interface NSObject (WebResourceLoadDelegate)

- (id)webView:(WebView *)sender identifierForInitialRequest:(NSURLRequest *)request fromDataSource:(WebDataSource *)dataSource;

- (NSURLRequest *)webView:(WebView *)sender resource:(id)identifier willSendRequest:(NSURLRequest *)request redirectResponse:(NSURLResponse *)redirectResponse fromDataSource:(WebDataSource *)dataSource;

- (void)webView:(WebView *)sender resource:(id)identifier didReceiveAuthenticationChallenge:(NSURLAuthenticationChallenge *)challenge fromDataSource:(WebDataSource *)dataSource;

- (void)webView:(WebView *)sender resource:(id)identifier didCancelAuthenticationChallenge:(NSURLAuthenticationChallenge *)challenge fromDataSource:(WebDataSource *)dataSource;

- (void)webView:(WebView *)sender resource:(id)identifier didReceiveResponse:(NSURLResponse *)response fromDataSource:(WebDataSource *)dataSource;

- (void)webView:(WebView *)sender resource:(id)identifier didReceiveContentLength:(WebNSInteger)length fromDataSource:(WebDataSource *)dataSource;

- (void)webView:(WebView *)sender resource:(id)identifier didFinishLoadingFromDataSource:(WebDataSource *)dataSource;

- (void)webView:(WebView *)sender resource:(id)identifier didFailLoadingWithError:(NSError *)error fromDataSource:(WebDataSource *)dataSource;

- (void)webView:(WebView *)sender plugInFailedWithError:(NSError *)error dataSource:(WebDataSource *)dataSource;

@end

#undef WebNSInteger
