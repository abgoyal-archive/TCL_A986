

#import <Foundation/Foundation.h>

@interface NSURLRequest (WebNSURLRequestExtras)

- (NSString *)_web_HTTPReferrer;
- (NSString *)_web_HTTPContentType;
- (BOOL)_web_isConditionalRequest;
@end

@interface NSMutableURLRequest (WebNSURLRequestExtras)

- (void)_web_setHTTPContentType:(NSString *)contentType;
- (void)_web_setHTTPReferrer:(NSString *)theReferrer;
- (void)_web_setHTTPUserAgent:(NSString *)theUserAgent;

@end
