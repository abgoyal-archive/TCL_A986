

#import <Cocoa/Cocoa.h>
#import <JavaScriptCore/WebKitAvailability.h>


@interface NSObject (WebPlugIn)

- (void)webPlugInInitialize;

- (void)webPlugInStart;

- (void)webPlugInStop;

- (void)webPlugInDestroy;

- (void)webPlugInSetIsSelected:(BOOL)isSelected;

- (id)objectForWebScript;

- (void)webPlugInMainResourceDidReceiveResponse:(NSURLResponse *)response WEBKIT_OBJC_METHOD_ANNOTATION(AVAILABLE_IN_WEBKIT_VERSION_4_0);

- (void)webPlugInMainResourceDidReceiveData:(NSData *)data WEBKIT_OBJC_METHOD_ANNOTATION(AVAILABLE_IN_WEBKIT_VERSION_4_0);

- (void)webPlugInMainResourceDidFailWithError:(NSError *)error WEBKIT_OBJC_METHOD_ANNOTATION(AVAILABLE_IN_WEBKIT_VERSION_4_0);

- (void)webPlugInMainResourceDidFinishLoading WEBKIT_OBJC_METHOD_ANNOTATION(AVAILABLE_IN_WEBKIT_VERSION_4_0);

@end
