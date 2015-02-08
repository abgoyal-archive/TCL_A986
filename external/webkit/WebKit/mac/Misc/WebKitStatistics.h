

#import <Foundation/Foundation.h>

@interface WebKitStatistics : NSObject

+ (int)webViewCount;

+ (int)frameCount;
+ (int)dataSourceCount;
+ (int)viewCount;
+ (int)HTMLRepresentationCount;
+ (int)bridgeCount;

@end
