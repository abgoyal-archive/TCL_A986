

#import <Foundation/Foundation.h>

@interface WebGeolocationMock : NSObject {
}

+ (void)setPosition:(double)latitude:(double)longitude:(double)accuracy;
+ (void)setError:(int)code:(NSString *)message;
@end
