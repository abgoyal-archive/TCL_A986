

#import <Foundation/Foundation.h>

@class NSFont;

@interface WebStringTruncator : NSObject

+ (NSString *)centerTruncateString:(NSString *)string toWidth:(float)maxWidth withFont:(NSFont *)font;

// Default font is [NSFont menuFontOfSize:0].
+ (NSString *)centerTruncateString:(NSString *)string toWidth:(float)maxWidth;

+ (NSString *)rightTruncateString:(NSString *)string toWidth:(float)maxWidth withFont:(NSFont *)font;

+ (float)widthOfString:(NSString *)string font:(NSFont *)font;

@end
