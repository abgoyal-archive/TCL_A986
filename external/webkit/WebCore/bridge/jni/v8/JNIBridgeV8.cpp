

#include "config.h"
#include "JNIBridgeV8.h"

using namespace JSC::Bindings;

JavaField::JavaField(JNIEnv* env, jobject aField)
{
    // Get field type
    jobject fieldType = callJNIMethod<jobject>(aField, "getType", "()Ljava/lang/Class;");
    jstring fieldTypeName = static_cast<jstring>(callJNIMethod<jobject>(fieldType, "getName", "()Ljava/lang/String;"));
    m_type = JavaString(env, fieldTypeName);
    m_JNIType = JNITypeFromClassName(m_type.UTF8String());

    // Get field name
    jstring fieldName = static_cast<jstring>(callJNIMethod<jobject>(aField, "getName", "()Ljava/lang/String;"));
    m_name = JavaString(env, fieldName);

    m_field = new JObjectWrapper(aField);
}
