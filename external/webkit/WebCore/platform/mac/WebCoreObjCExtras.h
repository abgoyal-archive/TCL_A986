

#include <objc/objc.h>

#ifdef __cplusplus
extern "C" {
#endif

void WebCoreObjCFinalizeOnMainThread(Class cls);

// The 'Class' that should be passed in here is the class of the
// object that implements the dealloc method that this function is called from.
bool WebCoreObjCScheduleDeallocateOnMainThread(Class cls, id object);

#ifdef __cplusplus
}
#endif
