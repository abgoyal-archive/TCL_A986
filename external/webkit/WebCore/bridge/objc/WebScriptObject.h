

#import <Foundation/Foundation.h>
#import "runtime_root.h"

@class WebUndefined;

@protocol WebScriptObject
+ (NSString *)webScriptNameForSelector:(SEL)aSelector;
+ (BOOL)isSelectorExcludedFromWebScript:(SEL)aSelector;
+ (NSString *)webScriptNameForKey:(const char *)name;
+ (BOOL)isKeyExcludedFromWebScript:(const char *)name;

+ (id)_convertValueToObjcValue:(JSC::JSValue)value originRootObject:(JSC::Bindings::RootObject*)originRootObject rootObject:(JSC::Bindings::RootObject*)rootObject;
- _initWithJSObject:(JSC::JSObject*)imp originRootObject:(PassRefPtr<JSC::Bindings::RootObject>)originRootObject rootObject:(PassRefPtr<JSC::Bindings::RootObject>)rootObject;
- (JSC::JSObject *)_imp;
@end

@protocol WebUndefined
+ (WebUndefined *)undefined;
@end
