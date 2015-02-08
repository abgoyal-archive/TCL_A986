

#import <Cocoa/Cocoa.h>

class LayoutTestController;

@interface PolicyDelegate : NSObject {
    BOOL permissiveDelegate;
    LayoutTestController* controllerToNotifyDone;
}

- (void)setPermissive:(BOOL)permissive;
- (void)setControllerToNotifyDone:(LayoutTestController*)controller;

@end
