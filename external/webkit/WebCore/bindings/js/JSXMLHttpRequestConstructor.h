

#ifndef JSXMLHttpRequestConstructor_h
#define JSXMLHttpRequestConstructor_h

#include "JSDOMBinding.h"

namespace WebCore {

class JSXMLHttpRequestConstructor : public DOMConstructorObject {
public:
    JSXMLHttpRequestConstructor(JSC::ExecState*, JSDOMGlobalObject*);
    static const JSC::ClassInfo s_info;
private:
    virtual JSC::ConstructType getConstructData(JSC::ConstructData&);
    virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
};

} // namespace WebCore

#endif // JSXMLHttpRequestConstructor_h
