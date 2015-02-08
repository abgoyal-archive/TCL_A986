

#import <AppKit/AppKit.h>

@class WebView;

@interface DumpRenderTreeWindow : NSWindow
{
}

// I'm not sure why we can't just use [NSApp windows]
+ (NSArray *)openWindows;

- (WebView *)webView;

- (void)startListeningForAcceleratedCompositingChanges;

@end
