

#include "config.h"
#include "ScriptStringImpl.h"

#include "V8Binding.h"

namespace WebCore {

ScriptStringImpl::ScriptStringImpl(const String& s)
{
    v8::HandleScope scope;
    m_handle.set(v8String(s));
}

ScriptStringImpl::ScriptStringImpl(const char* s)
{
    v8::HandleScope scope;
    m_handle.set(v8::String::New(s));
}

String ScriptStringImpl::toString() const
{
    return v8StringToWebCoreString(m_handle.get());
}

bool ScriptStringImpl::isNull() const
{
    return m_handle.get().IsEmpty();
}

size_t ScriptStringImpl::size() const
{
    return m_handle.get()->Length();
}

void ScriptStringImpl::append(const String& s)
{
    v8::HandleScope scope;
    if (m_handle.get().IsEmpty())
        m_handle.set(v8String(s));
    else
        m_handle.set(v8::String::Concat(m_handle.get(), v8String(s)));
}

} // namespace WebCore
