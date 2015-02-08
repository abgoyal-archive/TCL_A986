

#ifndef _WEB_SCRIPT_OBJECT_PRIVATE_H_
#define _WEB_SCRIPT_OBJECT_PRIVATE_H_

#import "WebScriptObject.h"
#import <runtime/JSValue.h>
#import <wtf/PassRefPtr.h>

namespace JSC {
    
    class JSObject;
    
    namespace Bindings {
        class RootObject;
    }
}
namespace WebCore {
    NSObject* getJSWrapper(JSC::JSObject*);
    void addJSWrapper(NSObject* wrapper, JSC::JSObject*);
    void removeJSWrapper(JSC::JSObject*);
    id createJSWrapper(JSC::JSObject*, PassRefPtr<JSC::Bindings::RootObject> origin, PassRefPtr<JSC::Bindings::RootObject> root);
}

@interface WebScriptObject (Private)
+ (id)_convertValueToObjcValue:(JSC::JSValue)value originRootObject:(JSC::Bindings::RootObject*)originRootObject rootObject:(JSC::Bindings::RootObject*)rootObject;
+ (id)scriptObjectForJSObject:(JSObjectRef)jsObject originRootObject:(JSC::Bindings::RootObject*)originRootObject rootObject:(JSC::Bindings::RootObject*)rootObject;
- (id)_init;
- (id)_initWithJSObject:(JSC::JSObject*)imp originRootObject:(PassRefPtr<JSC::Bindings::RootObject>)originRootObject rootObject:(PassRefPtr<JSC::Bindings::RootObject>)rootObject;
- (void)_setImp:(JSC::JSObject*)imp originRootObject:(PassRefPtr<JSC::Bindings::RootObject>)originRootObject rootObject:(PassRefPtr<JSC::Bindings::RootObject>)rootObject;
- (void)_setOriginRootObject:(PassRefPtr<JSC::Bindings::RootObject>)originRootObject andRootObject:(PassRefPtr<JSC::Bindings::RootObject>)rootObject;
- (void)_initializeScriptDOMNodeImp;
- (JSC::JSObject *)_imp;
- (BOOL)_hasImp;
- (JSC::Bindings::RootObject*)_rootObject;
- (JSC::Bindings::RootObject*)_originRootObject;
@end

@interface WebScriptObjectPrivate : NSObject
{
@public
    JSC::JSObject *imp;
    JSC::Bindings::RootObject* rootObject;
    JSC::Bindings::RootObject* originRootObject;
    BOOL isCreatedByDOMWrapper;
}
@end


#endif
