

#import <WebKit/WebHTMLRepresentation.h>

@protocol WebPluginManualLoader;

@interface WebHTMLRepresentation (WebPrivate)
- (void)_redirectDataToManualLoader:(id<WebPluginManualLoader>)manualLoader forPluginView:(NSView *)pluginView;
- (void)printDOMTree;
@end
