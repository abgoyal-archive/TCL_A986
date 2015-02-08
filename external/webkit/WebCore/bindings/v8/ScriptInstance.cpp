

#include "config.h"
#include "ScriptInstance.h"

#ifndef NDEBUG
#include "V8Proxy.h"
#endif
#include <wtf/Assertions.h>

namespace WebCore {

V8ScriptInstance::V8ScriptInstance()
{
}

V8ScriptInstance::V8ScriptInstance(v8::Handle<v8::Object> instance)
{
    set(instance);
}

V8ScriptInstance::~V8ScriptInstance()
{
    clear();
}

v8::Persistent<v8::Object> V8ScriptInstance::instance()
{
    return m_instance;
}

void V8ScriptInstance::clear()
{
    if (m_instance.IsEmpty())
        return;
#ifndef NDEBUG
    V8GCController::unregisterGlobalHandle(this, m_instance);
#endif
    m_instance.Dispose();
    m_instance.Clear();
}

void V8ScriptInstance::set(v8::Handle<v8::Object> instance)
{
    clear();
    if (instance.IsEmpty())
        return;

    m_instance = v8::Persistent<v8::Object>::New(instance);
#ifndef NDEBUG
    V8GCController::registerGlobalHandle(SCRIPTINSTANCE, this, m_instance);
#endif
}

} // namespace WebCore
