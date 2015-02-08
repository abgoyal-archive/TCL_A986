

#import <Foundation/Foundation.h>
#import <JavaScriptCore/JSBase.h>

@class DOMDocument;
@class DOMHTMLElement;
@class NSURLRequest;
@class WebArchive;
@class WebDataSource;
@class WebFramePrivate;
@class WebFrameView;
@class WebScriptObject;
@class WebView;

@interface WebFrame : NSObject
{
@private
    WebFramePrivate *_private;
}

- (id)initWithName:(NSString *)name webFrameView:(WebFrameView *)view webView:(WebView *)webView;

- (NSString *)name;

- (WebView *)webView;

- (WebFrameView *)frameView;

- (DOMDocument *)DOMDocument;

- (DOMHTMLElement *)frameElement;

- (void)loadRequest:(NSURLRequest *)request;

- (void)loadData:(NSData *)data MIMEType:(NSString *)MIMEType textEncodingName:(NSString *)encodingName baseURL:(NSURL *)URL;

- (void)loadHTMLString:(NSString *)string baseURL:(NSURL *)URL;

- (void)loadAlternateHTMLString:(NSString *)string baseURL:(NSURL *)baseURL forUnreachableURL:(NSURL *)unreachableURL;

- (void)loadArchive:(WebArchive *)archive;

- (WebDataSource *)dataSource;

- (WebDataSource *)provisionalDataSource;

- (void)stopLoading;

- (void)reload;

- (void)reloadFromOrigin;

- (WebFrame *)findFrameNamed:(NSString *)name;

- (WebFrame *)parentFrame;

- (NSArray *)childFrames;

- (WebScriptObject *)windowObject;

- (JSGlobalContextRef)globalContext;

@end
