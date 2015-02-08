

#import "WebTypesInternal.h"
#import <Cocoa/Cocoa.h>

@interface NSArray (WebNSArrayExtras)

-(NSNumber *)_webkit_numberAtIndex:(NSUInteger)index;
-(NSString *)_webkit_stringAtIndex:(NSUInteger)index;

@end

@interface NSMutableArray (WebNSArrayExtras)

- (void)_webkit_removeUselessMenuItemSeparators;

@end
