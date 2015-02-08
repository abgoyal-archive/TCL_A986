

#ifndef JNIBridge_h
#define JNIBridge_h

#if ENABLE(MAC_JAVA_BRIDGE)

#include "Bridge.h"
#include "JNIUtility.h"

#if USE(JSC)
#include "JavaStringJSC.h"
#elif USE(V8)
#include "JavaStringV8.h"
#endif

namespace JSC {

namespace Bindings {

typedef const char* RuntimeType;

class JavaString {
public:
    JavaString()
    {
        m_impl.init();
    }

    JavaString(JNIEnv* e, jstring s)
    {
        m_impl.init(e, s);
    }

    JavaString(jstring s)
    {
        m_impl.init(getJNIEnv(), s);
    }

    const char* UTF8String() const { return m_impl.UTF8String(); }
    const jchar* uchars() const { return m_impl.uchars(); }
    int length() const { return m_impl.length(); }
#if USE(JSC)
    operator UString() const { return m_impl.uString(); }
#endif

private:
    JavaStringImpl m_impl;
};

class JavaParameter {
public:
    JavaParameter() : m_JNIType(invalid_type) { }
    JavaParameter(JNIEnv*, jstring type);
    virtual ~JavaParameter() { }

    RuntimeType type() const { return m_type.UTF8String(); }
    JNIType getJNIType() const { return m_JNIType; }

private:
    JavaString m_type;
    JNIType m_JNIType;
};

class JavaMethod : public Method {
public:
    JavaMethod(JNIEnv*, jobject aMethod);
    ~JavaMethod();

    const JavaString& name() const { return m_name; }
    RuntimeType returnType() const { return m_returnType.UTF8String(); }
    JavaParameter* parameterAt(int i) const { return &m_parameters[i]; }
    int numParameters() const { return m_numParameters; }

    const char* signature() const;
    JNIType JNIReturnType() const;

    jmethodID methodID(jobject obj) const;

    bool isStatic() const { return m_isStatic; }

private:
    JavaParameter* m_parameters;
    int m_numParameters;
    JavaString m_name;
    mutable char* m_signature;
    JavaString m_returnType;
    JNIType m_JNIReturnType;
    mutable jmethodID m_methodID;
    bool m_isStatic;
};

} // namespace Bindings

} // namespace JSC

#endif // ENABLE(MAC_JAVA_BRIDGE)

#endif // JNIBridge_h
