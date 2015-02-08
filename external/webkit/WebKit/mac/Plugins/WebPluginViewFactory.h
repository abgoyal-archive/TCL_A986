

#import <Cocoa/Cocoa.h>
#import <JavaScriptCore/WebKitAvailability.h>

extern NSString *WebPlugInBaseURLKey;

extern NSString *WebPlugInAttributesKey;

extern NSString *WebPlugInContainerKey;

extern NSString *WebPlugInContainingElementKey;

extern NSString *WebPlugInShouldLoadMainResourceKey AVAILABLE_IN_WEBKIT_VERSION_4_0;


@protocol WebPlugInViewFactory <NSObject>

+ (NSView *)plugInViewWithArguments:(NSDictionary *)arguments;

@end
