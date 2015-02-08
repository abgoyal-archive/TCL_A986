

#ifndef JavaClassJSC_h
#define JavaClassJSC_h

#if ENABLE(MAC_JAVA_BRIDGE)

#include "JNIBridgeJSC.h"
#include <wtf/HashMap.h>

namespace JSC {

namespace Bindings {

class JavaClass : public Class {
public:
    JavaClass(jobject);
    ~JavaClass();

    virtual MethodList methodsNamed(const Identifier&, Instance*) const;
    virtual Field* fieldNamed(const Identifier&, Instance*) const;

    bool isNumberClass() const;
    bool isBooleanClass() const;
    bool isStringClass() const;

private:
    const char* m_name;
    FieldMap m_fields;
    MethodListMap m_methods;
};

} // namespace Bindings

} // namespace JSC

#endif // ENABLE(MAC_JAVA_BRIDGE)

#endif // JavaClassJSC_h
