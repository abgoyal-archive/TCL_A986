

#ifndef JavaClassV8_h
#define JavaClassV8_h

#include "JNIBridgeV8.h"
#include "PlatformString.h"
#include "StringHash.h"
#include <wtf/HashMap.h>
#include <wtf/Vector.h>

namespace JSC {

namespace Bindings {

typedef Vector<JavaMethod*> MethodList;
typedef HashMap<WebCore::String, MethodList*> MethodListMap;
typedef HashMap<WebCore::String, JavaField*> FieldMap;

class JavaClass {
public:
    JavaClass(jobject anInstance);
    ~JavaClass();

    MethodList methodsNamed(const char* name) const;
    JavaField* fieldNamed(const char* name) const;

private:
    const char* m_name;
    MethodListMap m_methods;
    FieldMap m_fields;
};

} // namespace Bindings

} // namespace JSC

#endif // JavaClassV8_h
