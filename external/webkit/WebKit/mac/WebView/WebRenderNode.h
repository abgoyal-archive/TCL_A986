

#import <Foundation/Foundation.h>

@class WebFrame;

@interface WebRenderNode : NSObject
{
    NSArray *children;
    NSString *name;
    NSRect rect;
    NSPoint absolutePosition;
}

- (id)initWithWebFrame:(WebFrame *)frame;

- (NSArray *)children;

- (NSString *)name;
- (NSString *)positionString;
- (NSString *)absolutePositionString;
- (NSString *)widthString;
- (NSString *)heightString;

@end
