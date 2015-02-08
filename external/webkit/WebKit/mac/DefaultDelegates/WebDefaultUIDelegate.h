

#import <Foundation/Foundation.h>

@interface WebDefaultUIDelegate : NSObject
{
    IBOutlet NSMenu *defaultMenu;
}
+ (WebDefaultUIDelegate *)sharedUIDelegate;
@end
