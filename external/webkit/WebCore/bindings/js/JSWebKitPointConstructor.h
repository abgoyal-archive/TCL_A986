

#ifndef JSWebKitPointConstructor_h
#define JSWebKitPointConstructor_h

#include "JSDOMBinding.h"
#include "JSDocument.h"

namespace WebCore {

class JSWebKitPointConstructor : public DOMConstructorObject {
public:
    JSWebKitPointConstructor(JSC::ExecState*, JSDOMGlobalObject*);
    static const JSC::ClassInfo s_info;

private:
    virtual JSC::ConstructType getConstructData(JSC::ConstructData&);
    virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
};

}

#endif // JSWebKitPointConstructor_h
