

#ifndef JNIBridgeJSC_h
#define JNIBridgeJSC_h

#if ENABLE(MAC_JAVA_BRIDGE)

#include "Bridge.h"
#include "JNIBridge.h"
#include <JavaVM/jni.h>

namespace JSC {

namespace Bindings {

class JavaField : public Field {
public:
    JavaField(JNIEnv*, jobject aField);

    virtual JSValue valueFromInstance(ExecState*, const Instance*) const;
    virtual void setValueToInstance(ExecState*, const Instance*, JSValue) const;

    const JavaString& name() const { return m_name; }
    virtual RuntimeType type() const { return m_type.UTF8String(); }

    JNIType getJNIType() const { return m_JNIType; }

private:
    void dispatchSetValueToInstance(ExecState*, const JavaInstance*, jvalue, const char* name, const char* sig) const;
    jvalue dispatchValueFromInstance(ExecState*, const JavaInstance*, const char* name, const char* sig, JNIType returnType) const;

    JavaString m_name;
    JavaString m_type;
    JNIType m_JNIType;
    RefPtr<JObjectWrapper> m_field;
};

class JavaArray : public Array {
public:
    JavaArray(jobject array, const char* type, PassRefPtr<RootObject>);
    virtual ~JavaArray();

    RootObject* rootObject() const;

    virtual void setValueAt(ExecState*, unsigned int index, JSValue) const;
    virtual JSValue valueAt(ExecState*, unsigned int index) const;
    virtual unsigned int getLength() const;

    jobject javaArray() const { return m_array->m_instance; }

    static JSValue convertJObjectToArray(ExecState*, jobject, const char* type, PassRefPtr<RootObject>);

private:
    RefPtr<JObjectWrapper> m_array;
    unsigned int m_length;
    const char* m_type;
};

} // namespace Bindings

} // namespace JSC

#endif // ENABLE(MAC_JAVA_BRIDGE)

#endif // JNIBridge_h
