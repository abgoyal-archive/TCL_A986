

#include "config.h"
#include "JavaClassJSC.h"

#if ENABLE(MAC_JAVA_BRIDGE)

#include "JNIUtility.h"
#include "JSDOMWindow.h"
#include <runtime/Identifier.h>
#include <runtime/JSLock.h>

using namespace JSC::Bindings;

JavaClass::JavaClass(jobject anInstance)
{
    jobject aClass = callJNIMethod<jobject>(anInstance, "getClass", "()Ljava/lang/Class;");

    if (!aClass) {
        fprintf(stderr, "%s:  unable to call getClass on instance %p\n", __PRETTY_FUNCTION__, anInstance);
        m_name = fastStrDup("<Unknown>");
        return;
    }

    if (jstring className = (jstring)callJNIMethod<jobject>(aClass, "getName", "()Ljava/lang/String;")) {
        const char* classNameC = getCharactersFromJString(className);
        m_name = fastStrDup(classNameC);
        releaseCharactersForJString(className, classNameC);
    } else
        m_name = fastStrDup("<Unknown>");

    int i;
    JNIEnv* env = getJNIEnv();

    // Get the fields
    if (jarray fields = (jarray)callJNIMethod<jobject>(aClass, "getFields", "()[Ljava/lang/reflect/Field;")) {
        int numFields = env->GetArrayLength(fields);
        for (i = 0; i < numFields; i++) {
            jobject aJField = env->GetObjectArrayElement((jobjectArray)fields, i);
            JavaField* aField = new JavaField(env, aJField); // deleted in the JavaClass destructor
            {
                JSLock lock(SilenceAssertionsOnly);
                m_fields.set(((UString)aField->name()).rep(), aField);
            }
            env->DeleteLocalRef(aJField);
        }
        env->DeleteLocalRef(fields);
    }

    // Get the methods
    if (jarray methods = (jarray)callJNIMethod<jobject>(aClass, "getMethods", "()[Ljava/lang/reflect/Method;")) {
        int numMethods = env->GetArrayLength(methods);
        for (i = 0; i < numMethods; i++) {
            jobject aJMethod = env->GetObjectArrayElement((jobjectArray)methods, i);
            JavaMethod* aMethod = new JavaMethod(env, aJMethod); // deleted in the JavaClass destructor
            MethodList* methodList;
            {
                JSLock lock(SilenceAssertionsOnly);

                methodList = m_methods.get(((UString)aMethod->name()).rep());
                if (!methodList) {
                    methodList = new MethodList();
                    m_methods.set(((UString)aMethod->name()).rep(), methodList);
                }
            }
            methodList->append(aMethod);
            env->DeleteLocalRef(aJMethod);
        }
        env->DeleteLocalRef(methods);
    }

    env->DeleteLocalRef(aClass);
}

JavaClass::~JavaClass()
{
    fastFree(const_cast<char*>(m_name));

    JSLock lock(SilenceAssertionsOnly);

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

MethodList JavaClass::methodsNamed(const Identifier& identifier, Instance*) const
{
    MethodList* methodList = m_methods.get(identifier.ustring().rep());

    if (methodList)
        return *methodList;
    return MethodList();
}

Field* JavaClass::fieldNamed(const Identifier& identifier, Instance*) const
{
    return m_fields.get(identifier.ustring().rep());
}

bool JavaClass::isNumberClass() const
{
    return (!strcmp(m_name, "java.lang.Byte")
        || !strcmp(m_name, "java.lang.Short")
        || !strcmp(m_name, "java.lang.Integer")
        || !strcmp(m_name, "java.lang.Long")
        || !strcmp(m_name, "java.lang.Float")
        || !strcmp(m_name, "java.lang.Double"));
}

bool JavaClass::isBooleanClass() const
{
    return !strcmp(m_name, "java.lang.Boolean");
}

bool JavaClass::isStringClass() const
{
    return !strcmp(m_name, "java.lang.String");
}

#endif // ENABLE(MAC_JAVA_BRIDGE)
