

#import <Cocoa/Cocoa.h>


@interface ObjCPlugin : NSObject
{
    BOOL throwOnDealloc;
}

- (void)removeBridgeRestrictions:(id)container;

@end
