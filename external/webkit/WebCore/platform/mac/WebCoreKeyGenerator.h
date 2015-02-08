

@interface WebCoreKeyGenerator : NSObject

+ (WebCoreKeyGenerator *)sharedGenerator;
- (NSArray *)strengthMenuItemTitles;
- (NSString *)signedPublicKeyAndChallengeStringWithStrengthIndex:(unsigned)index challenge:(NSString *)challenge pageURL:(NSURL *)pageURL;

@end
