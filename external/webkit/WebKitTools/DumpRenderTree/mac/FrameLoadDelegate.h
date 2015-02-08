

#import <Foundation/Foundation.h>

class AccessibilityController;
class GCController;

@interface FrameLoadDelegate : NSObject
{
    AccessibilityController* accessibilityController;
    GCController* gcController;
}

- (void)resetToConsistentState;

@end
