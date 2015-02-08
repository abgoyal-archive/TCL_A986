

#include <ApplicationServices/ApplicationServices.h>
#include <objc/objc.h>

#ifdef __OBJC__
#include <Foundation/Foundation.h>
@class NSString;
#else
typedef struct NSString NSString;
#endif

#ifdef __cplusplus
extern "C" {
#endif

BOOL stringIsCaseInsensitiveEqualToString(NSString *first, NSString *second);
BOOL hasCaseInsensitiveSuffix(NSString *string, NSString *suffix);
BOOL hasCaseInsensitiveSubstring(NSString *string, NSString *substring);
NSString *filenameByFixingIllegalCharacters(NSString *string);
CFStringEncoding stringEncodingForResource(Handle resource);

#ifdef __cplusplus
}
#endif
