

#import <WebKit/WebKit.h>
#include <HIToolbox/HIView.h>

@interface HIViewAdapter : NSObject

+ (void)bindHIViewToNSView:(HIViewRef)hiView nsView:(NSView*)nsView;
+ (void)unbindNSView:(NSView*)nsView;
+ (HIViewRef)getHIViewForNSView:(NSView*)inView;

@end
