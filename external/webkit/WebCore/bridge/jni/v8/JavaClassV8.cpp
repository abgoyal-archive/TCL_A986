

#include "config.h"
#include "JavaClassV8.h"


using namespace JSC::Bindings;

JavaClass::JavaClass(jobject anInstance)
{
    jobject aClass = callJNIMethod<jobject>(anInstance, "getClass", "()Ljava/lang/Class;");

    if (!aClass) {
        fprintf(stderr, "%s:  unable to call getClass on instance %p\n", __PRETTY_FUNCTION__, anInstance);
        return;
    }

    jstring className = static_cast<jstring>(callJNIMethod<jobject>(aClass, "getName", "()Ljava/lang/String;"));
    const char* classNameC = getCharactersFromJString(className);
    m_name = strdup(classNameC);
    releaseCharactersForJString(className, classNameC);

    int i;
    JNIEnv* env = getJNIEnv();

    // Get the fields
    jarray fields = static_cast<jarray>(callJNIMethod<jobject>(aClass, "getFields", "()[Ljava/lang/reflect/Field;"));
    int numFields = env->GetArrayLength(fields);
    for (i = 0; i < numFields; i++) {
        jobject aJField = env->GetObjectArrayElement(static_cast<jobjectArray>(fields), i);
        JavaField* aField = new JavaField(env, aJField); // deleted in the JavaClass destructor
        {
            m_fields.set(aField->name().UTF8String(), aField);
        }
        env->DeleteLocalRef(aJField);
    }

    // Get the methods
    jarray methods = static_cast<jarray>(callJNIMethod<jobject>(aClass, "getMethods", "()[Ljava/lang/reflect/Method;"));
    int numMethods = env->GetArrayLength(methods);
    for (i = 0; i < numMethods; i++) {
        jobject aJMethod = env->GetObjectArrayElement(static_cast<jobjectArray>(methods), i);
        JavaMethod* aMethod = new JavaMethod(env, aJMethod); // deleted in the JavaClass destructor
        MethodList* methodList;
        {
            methodList = m_methods.get(aMethod->name().UTF8String());
            if (!methodList) {
                methodList = new MethodList();
                m_methods.set(aMethod->name().UTF8String(), methodList);
            }
        }
        methodList->append(aMethod);
        env->DeleteLocalRef(aJMethod);
    }
    env->DeleteLocalRef(fields);
    env->DeleteLocalRef(methods);
    env->DeleteLocalRef(aClass);
}

JavaClass::~JavaClass()
{
    free(const_cast<char*>(m_name));

    deleteAllValues(m_fields);
    m_fields.clear();

    MethodListMap::const_iterator end = m_methods.end();
    for (MethodListMap::const_iterator it = m_methods.begin(); it != end; ++it) {
        const MethodList* methodList = it->second;
        deleteAllValues(*methodList);
        delete methodList;
    }
    m_methods.clear();
}

MethodList JavaClass::methodsNamed(const char* name) const
{
    MethodList* methodList = m_methods.get(name);

    if (methodList)
        return *methodList;
    return MethodList();
}

JavaField* JavaClass::fieldNamed(const char* name) const
{
    return m_fields.get(name);
}
