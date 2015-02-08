

#import <Foundation/Foundation.h>

@interface NSDictionary (WebNSDictionaryExtras)
- (BOOL)_webkit_boolForKey:(id)key;
- (int)_webkit_intForKey:(id)key;
- (NSString *)_webkit_stringForKey:(id)key; // Returns nil if the value is not an NSString.
- (NSArray *)_webkit_arrayForKey:(id)key; // Returns nil if the value is not an NSArray.

// Searches for the full MIME type, then the prefix (e.g., "text/" for "text/html")
- (id)_webkit_objectForMIMEType:(NSString *)MIMEType;
@end

@interface NSMutableDictionary (WebNSDictionaryExtras)
- (void)_webkit_setObject:(id)object forUncopiedKey:(id)key;
- (void)_webkit_setInt:(int)value forKey:(id)key;
- (void)_webkit_setFloat:(float)value forKey:(id)key;
- (void)_webkit_setBool:(BOOL)value forKey:(id)key;
- (void)_webkit_setUnsignedLongLong:(unsigned long long)value forKey:(id)key;
@end
