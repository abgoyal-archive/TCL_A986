

#ifndef JavaInstanceJSC_h
#define JavaInstanceJSC_h

#if ENABLE(MAC_JAVA_BRIDGE)

#include "Bridge.h"
#include "runtime_root.h"

#include <JavaVM/jni.h>

namespace JSC {

namespace Bindings {

class JavaClass;

class JObjectWrapper {
friend class RefPtr<JObjectWrapper>;
friend class JavaArray;
friend class JavaField;
friend class JavaInstance;
friend class JavaMethod;

public:
    jobject instance() const { return m_instance; }
    void setInstance(jobject instance) { m_instance = instance; }

    void ref() { m_refCount++; }
    void deref()
    {
        if (!(--m_refCount))
            delete this;
    }

protected:
    JObjectWrapper(jobject instance);
    ~JObjectWrapper();

    jobject m_instance;

private:
    JNIEnv* m_env;
    unsigned int m_refCount;
};

class JavaInstance : public Instance {
public:
    static PassRefPtr<JavaInstance> create(jobject instance, PassRefPtr<RootObject> rootObject)
    {
        return adoptRef(new JavaInstance(instance, rootObject));
    }

    ~JavaInstance();

    virtual Class* getClass() const;

    virtual JSValue valueOf(ExecState*) const;
    virtual JSValue defaultValue(ExecState*, PreferredPrimitiveType) const;

    virtual JSValue invokeMethod(ExecState* exec, const MethodList& method, const ArgList& args);

    jobject javaInstance() const { return m_instance->m_instance; }

    JSValue stringValue(ExecState*) const;
    JSValue numberValue(ExecState*) const;
    JSValue booleanValue() const;

protected:
    JavaInstance(jobject instance, PassRefPtr<RootObject>);
    virtual void virtualBegin();
    virtual void virtualEnd();

    RefPtr<JObjectWrapper> m_instance;
    mutable JavaClass* m_class;
};

} // namespace Bindings

} // namespace JSC

#endif // ENABLE(MAC_JAVA_BRIDGE)

#endif // JavaInstanceJSC_h
