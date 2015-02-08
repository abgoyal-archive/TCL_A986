

#include "config.h"
#include "BoundObject.h"

#include "V8Proxy.h"

namespace WebKit {

BoundObject::BoundObject(v8::Handle<v8::Context> context, void* v8This, const char* objectName)
    : m_objectName(objectName)
    , m_context(context)
    , m_v8This(v8This)
{
    v8::Context::Scope contextScope(context);
    v8::Local<v8::FunctionTemplate> localTemplate = v8::FunctionTemplate::New(WebCore::V8Proxy::checkNewLegal);
    m_hostTemplate = v8::Persistent<v8::FunctionTemplate>::New(localTemplate);
    m_hostTemplate->SetClassName(v8::String::New(objectName));
}

BoundObject::~BoundObject()
{
    m_hostTemplate.Dispose();
}

void BoundObject::addProtoFunction(const char* name, v8::InvocationCallback callback)
{
    v8::Context::Scope contextScope(m_context);
    v8::Local<v8::Signature> signature = v8::Signature::New(m_hostTemplate);
    v8::Local<v8::ObjectTemplate> proto = m_hostTemplate->PrototypeTemplate();
    v8::Local<v8::External> v8This = v8::External::New(m_v8This);
    proto->Set(
        v8::String::New(name),
        v8::FunctionTemplate::New(
            callback,
            v8This,
            signature),
        static_cast<v8::PropertyAttribute>(v8::DontDelete));
}

void BoundObject::build()
{
    v8::Context::Scope contextScope(m_context);
    v8::Local<v8::Function> constructor = m_hostTemplate->GetFunction();
    v8::Local<v8::Object> boundObject = WebCore::SafeAllocation::newInstance(constructor);

    v8::Handle<v8::Object> global = m_context->Global();
    global->Set(v8::String::New(m_objectName), boundObject);
}

} // namespace WebKit
