

#import <Cocoa/Cocoa.h>

#import <WebKit/WebDocument.h>

@class NSMutableURLRequest;
@class NSURLConnection;
@class NSURLRequest;
@class NSURLResponse;
@class WebArchive;
@class WebDataSourcePrivate;
@class WebFrame;
@class WebResource;

@interface WebDataSource : NSObject
{
@private
    WebDataSourcePrivate *_private;
}

- (id)initWithRequest:(NSURLRequest *)request;

- (NSData *)data;

- (id <WebDocumentRepresentation>)representation;

- (WebFrame *)webFrame;

- (NSURLRequest *)initialRequest;

- (NSMutableURLRequest *)request;

- (NSURLResponse *)response;

- (NSString *)textEncodingName;

- (BOOL)isLoading;

- (NSString *)pageTitle;

- (NSURL *)unreachableURL;

- (WebArchive *)webArchive;

- (WebResource *)mainResource;

- (NSArray *)subresources;

- (WebResource *)subresourceForURL:(NSURL *)URL;

- (void)addSubresource:(WebResource *)subresource;

@end
