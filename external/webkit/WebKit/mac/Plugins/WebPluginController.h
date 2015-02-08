

#import <WebKit/WebBasePluginPackage.h>
#import <WebKit/WebPluginContainerCheck.h>

@class WebFrame;
@class WebHTMLView;
@class WebPluginPackage;
@class WebView;
@class WebDataSource;

@interface WebPluginController : NSObject <WebPluginManualLoader, WebPluginContainerCheckController>
{
    NSView *_documentView;
    WebDataSource *_dataSource;
    NSMutableArray *_views;
    BOOL _started;
    NSMutableSet *_checksInProgress;
}

+ (NSView *)plugInViewWithArguments:(NSDictionary *)arguments fromPluginPackage:(WebPluginPackage *)plugin;
+ (BOOL)isPlugInView:(NSView *)view;

- (id)initWithDocumentView:(NSView *)view;

- (void)setDataSource:(WebDataSource *)dataSource;

- (void)addPlugin:(NSView *)view;
- (void)destroyPlugin:(NSView *)view;

- (void)startAllPlugins;
- (void)stopAllPlugins;
- (void)destroyAllPlugins;

- (WebFrame *)webFrame;
- (WebView *)webView;

- (NSString *)URLPolicyCheckReferrer;

@end
