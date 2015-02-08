
 
#import <WebKit/WebDownload.h>
 
@interface WebDownload (WebDownloadCreation)
+(id)_downloadWithLoadingConnection:(NSURLConnection *)connection
                            request:(NSURLRequest *)request
                           response:(NSURLResponse *)r
                           delegate:(id)delegate
                              proxy:(id)proxy;

+(id)_downloadWithRequest:(NSURLRequest *)request
                 delegate:(id)delegate
                directory:(NSString *)directory;
@end
