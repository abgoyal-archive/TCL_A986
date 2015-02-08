

@interface WebCache : NSObject
{
}

+ (NSArray *)statistics;
+ (void)empty;
+ (void)setDisabled:(BOOL)disabled;
+ (BOOL)isDisabled;

@end
