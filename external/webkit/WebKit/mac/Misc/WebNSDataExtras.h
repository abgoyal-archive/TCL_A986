

#import <Foundation/Foundation.h>

#import "WebTypesInternal.h"

#define WEB_GUESS_MIME_TYPE_PEEK_LENGTH 1024

@interface NSData (WebNSDataExtras)

-(BOOL)_web_isCaseInsensitiveEqualToCString:(const char *)string;
-(NSMutableDictionary *)_webkit_parseRFC822HeaderFields;
- (BOOL)_web_startsWithBlankLine;
- (NSInteger)_web_locationAfterFirstBlankLine;

@end
