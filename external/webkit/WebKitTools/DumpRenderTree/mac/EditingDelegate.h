
 
#import <Cocoa/Cocoa.h>

@interface EditingDelegate : NSObject
{
    BOOL acceptsEditing;
}

- (void)setAcceptsEditing:(BOOL)newAcceptsEditing;

@end
