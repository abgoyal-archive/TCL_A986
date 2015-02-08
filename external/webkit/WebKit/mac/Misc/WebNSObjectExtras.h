

#import <Foundation/Foundation.h>
#import <objc/objc-class.h>
#import <objc/objc.h>

// Use WebCFAutorelease to return an object made by a CoreFoundation
// "create" or "copy" function as an autoreleased and garbage collected
// object. CF objects need to be "made collectable" for autorelease to work
// properly under GC.
static inline id WebCFAutorelease(CFTypeRef obj)
{
    if (obj)
        CFMakeCollectable(obj);
    [(id)obj autorelease];
    return (id)obj;
}

#if !(defined(OBJC_API_VERSION) && OBJC_API_VERSION > 0)

static inline IMP method_setImplementation(Method m, IMP i)
{
    IMP oi = m->method_imp;
    m->method_imp = i;
    return oi;
}

#endif

@interface NSObject (WebNSObjectExtras)
- (id)_webkit_invokeOnMainThread;
@end
