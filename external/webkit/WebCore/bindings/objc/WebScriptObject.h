

#import <Foundation/Foundation.h>
#import <JavaScriptCore/JSBase.h>
#import <JavaScriptCore/WebKitAvailability.h>

#if WEBKIT_VERSION_MAX_ALLOWED >= WEBKIT_VERSION_1_3

// NSObject (WebScripting) -----------------------------------------------------

@interface NSObject (WebScripting)

+ (NSString *)webScriptNameForSelector:(SEL)selector;

+ (BOOL)isSelectorExcludedFromWebScript:(SEL)selector;

+ (NSString *)webScriptNameForKey:(const char *)name;

+ (BOOL)isKeyExcludedFromWebScript:(const char *)name;

- (id)invokeUndefinedMethodFromWebScript:(NSString *)name withArguments:(NSArray *)arguments;

- (id)invokeDefaultMethodWithArguments:(NSArray *)arguments;

- (void)finalizeForWebScript;

@end


// WebScriptObject --------------------------------------------------

@class WebScriptObjectPrivate;
@class WebFrame;

@interface WebScriptObject : NSObject
{
    WebScriptObjectPrivate *_private;
}

+ (BOOL)throwException:(NSString *)exceptionMessage;

- (JSObjectRef)JSObject WEBKIT_OBJC_METHOD_ANNOTATION(AVAILABLE_WEBKIT_VERSION_3_0_AND_LATER);

- (id)callWebScriptMethod:(NSString *)name withArguments:(NSArray *)arguments;

- (id)evaluateWebScript:(NSString *)script;

- (void)removeWebScriptKey:(NSString *)name;

- (NSString *)stringRepresentation;

- (id)webScriptValueAtIndex:(unsigned)index;

- (void)setWebScriptValueAtIndex:(unsigned)index value:(id)value;

- (void)setException:(NSString *)description;

@end


// WebUndefined --------------------------------------------------------------

@interface WebUndefined : NSObject <NSCoding, NSCopying>

+ (WebUndefined *)undefined;

@end

#endif
