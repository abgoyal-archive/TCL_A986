

#import <Foundation/Foundation.h>

// FIXME: Change method names back to _web_ from _webkit_ when identically-named
// methods are no longer present in Foundation.

@interface NSURL (WebNSURLExtras)

+ (NSURL *)_web_URLWithUserTypedString:(NSString *)string;
+ (NSURL *)_web_URLWithUserTypedString:(NSString *)string relativeToURL:(NSURL *)URL;

+ (NSURL *)_web_URLWithDataAsString:(NSString *)string;
+ (NSURL *)_web_URLWithDataAsString:(NSString *)string relativeToURL:(NSURL *)baseURL;

+ (NSURL *)_web_URLWithData:(NSData *)data;
+ (NSURL *)_web_URLWithData:(NSData *)data relativeToURL:(NSURL *)baseURL;

- (NSURL *)_web_URLWithLowercasedScheme;

- (NSData *)_web_originalData;
- (NSString *)_web_originalDataAsString;
- (const char *)_web_URLCString;

- (NSData *)_web_hostData;
- (NSString *)_web_hostString;

- (NSString *)_web_userVisibleString;

- (BOOL)_web_isEmpty;

- (NSURL *)_webkit_canonicalize;
- (NSURL *)_webkit_URLByRemovingFragment;
- (NSURL *)_webkit_URLByRemovingResourceSpecifier;
- (NSURL *)_web_URLByRemovingUserInfo;

- (BOOL)_webkit_isJavaScriptURL;
- (BOOL)_webkit_isFileURL;
- (NSString *)_webkit_scriptIfJavaScriptURL;
- (BOOL)_webkit_isFTPDirectoryURL;

- (BOOL)_webkit_shouldLoadAsEmptyDocument;

- (NSString *)_webkit_suggestedFilenameWithMIMEType:(NSString *)MIMEType;

@end

@interface NSString (WebNSURLExtras)

- (BOOL)_web_isUserVisibleURL;

- (BOOL)_web_hostNameNeedsDecodingWithRange:(NSRange)range; // returns NO if decodeHostNameWithRange: would return nil, but more efficient
- (BOOL)_web_hostNameNeedsEncodingWithRange:(NSRange)range; // returns NO if encodeHostNameWithRange: would return nil, but more efficient

- (NSString *)_web_decodeHostNameWithRange:(NSRange)range; // turns funny-looking ASCII form into Unicode, returns nil if no decoding needed
- (NSString *)_web_encodeHostNameWithRange:(NSRange)range; // turns Unicode into funny-looking ASCII form, returns nil if no decoding needed

- (NSString *)_web_decodeHostName; // turns funny-looking ASCII form into Unicode, returns self if no decoding needed, convenient cover
- (NSString *)_web_encodeHostName; // turns Unicode into funny-looking ASCII form, returns self if no decoding needed, convenient cover

- (BOOL)_webkit_isJavaScriptURL;
- (BOOL)_webkit_isFTPDirectoryURL;
- (BOOL)_webkit_isFileURL;
- (BOOL)_webkit_looksLikeAbsoluteURL;
- (NSRange)_webkit_rangeOfURLScheme;
- (NSString *)_webkit_URLFragment;
- (NSString *)_webkit_scriptIfJavaScriptURL;

@end
