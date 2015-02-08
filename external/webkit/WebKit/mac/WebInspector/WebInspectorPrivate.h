

// This header contains the WebInspector SPI.

#import "WebInspector.h"

@interface WebInspector (WebPrivate)
- (void)evaluateInFrontend:(id)sender callId:(long)callId script:(NSString *)script;
@end
