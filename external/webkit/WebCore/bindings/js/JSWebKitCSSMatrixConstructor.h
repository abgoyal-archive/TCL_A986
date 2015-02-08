

#ifndef JSWebKitCSSMatrixConstructor_h
#define JSWebKitCSSMatrixConstructor_h

#include "JSDOMBinding.h"
#include "JSDocument.h"

namespace WebCore {

class JSWebKitCSSMatrixConstructor : public DOMConstructorObject {
public:
    JSWebKitCSSMatrixConstructor(JSC::ExecState*, JSDOMGlobalObject*);
    static const JSC::ClassInfo s_info;

private:
    virtual JSC::ConstructType getConstructData(JSC::ConstructData&);
    virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
};

}

#endif // JSWebKitCSSMatrixConstructor_h
