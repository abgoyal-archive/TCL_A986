

#import <Cocoa/Cocoa.h>


@interface NSObject (WebPlugInContainer)

- (void)webPlugInContainerLoadRequest:(NSURLRequest *)request inFrame:(NSString *)target;

- (void)webPlugInContainerShowStatus:(NSString *)message;

- (NSColor *)webPlugInContainerSelectionColor;

- (WebFrame *)webFrame;

@end
