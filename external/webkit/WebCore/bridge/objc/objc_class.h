

#ifndef KJS_BINDINGS_OBJC_CLASS_H
#define KJS_BINDINGS_OBJC_CLASS_H

#include "objc_runtime.h"

namespace JSC {
namespace Bindings {

class ObjcClass : public Class
{
protected:
    ObjcClass (ClassStructPtr aClass); // Use classForIsA to create an ObjcClass.
    
public:
    // Return the cached ObjC of the specified name.
    static ObjcClass *classForIsA(ClassStructPtr);
    
    virtual MethodList methodsNamed(const Identifier&, Instance *instance) const;
    virtual Field *fieldNamed(const Identifier&, Instance *instance) const;

    virtual JSValue fallbackObject(ExecState *exec, Instance *instance, const Identifier &propertyName);
    
    ClassStructPtr isa() { return _isa; }
    
private:
    ClassStructPtr _isa;
    RetainPtr<CFMutableDictionaryRef> _methods;
    RetainPtr<CFMutableDictionaryRef> _fields;
};

} // namespace Bindings
} // namespace JSC

#endif
