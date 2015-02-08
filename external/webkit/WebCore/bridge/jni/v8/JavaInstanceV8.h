

#ifndef JavaInstanceV8_h
#define JavaInstanceV8_h

#include "npruntime.h"

#include <JavaVM/jni.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

using namespace WTF;

namespace JSC {

namespace Bindings {

class JavaClass;

class JObjectWrapper {
friend class RefPtr<JObjectWrapper>;
friend class JavaField;
friend class JavaInstance;

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
    JObjectWrapper(jobject);
    ~JObjectWrapper();

    jobject m_instance;

private:
    JNIEnv* m_env;
    unsigned int m_refCount;
};

class JavaInstance : public RefCounted<JavaInstance> {
public:
    JavaInstance(jobject instance);
    virtual ~JavaInstance();

    JavaClass* getClass() const;

    bool invokeMethod(const char* name, const NPVariant* args, int argsCount, NPVariant* result);

    jobject javaInstance() const { return m_instance->m_instance; }

    // These functions are called before and after the main entry points into
    // the native implementations.  They can be used to establish and cleanup
    // any needed state.
    void begin() { virtualBegin(); }
    void end() { virtualEnd(); }

protected:
    RefPtr<JObjectWrapper> m_instance;
    mutable JavaClass* m_class;

    virtual void virtualBegin();
    virtual void virtualEnd();
};

} // namespace Bindings

} // namespace JSC

#endif // JavaInstanceV8_h
