

#ifndef JNIBridgeV8_h
#define JNIBridgeV8_h

#include "JNIBridge.h" // For JavaString
#include "JavaInstanceV8.h" // For JObjectWrapper

namespace JSC {

namespace Bindings {

class JavaField {
public:
    JavaField(JNIEnv*, jobject aField);

    const JavaString& name() const { return m_name; }
    const char* type() const { return m_type.UTF8String(); }

    JNIType getJNIType() const { return m_JNIType; }

private:
    JavaString m_name;
    JavaString m_type;
    JNIType m_JNIType;
    RefPtr<JObjectWrapper> m_field;
};

} // namespace Bindings

} // namespace JSC

#endif // JNIBridgeV8_h
