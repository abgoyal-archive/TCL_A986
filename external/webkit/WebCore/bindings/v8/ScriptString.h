

#ifndef ScriptString_h
#define ScriptString_h

#include "PlatformString.h"
#include "ScriptStringImpl.h"
#include "V8Binding.h"

namespace WebCore {

class ScriptString {
public:
    ScriptString() : m_impl(0) {}
    ScriptString(const String& s) : m_impl(ScriptStringImpl::create(s)) {}
    ScriptString(const char* s) : m_impl(ScriptStringImpl::create(s)) {}

    operator String() const { return m_impl->toString(); }

    bool isNull() const { return !m_impl.get() || m_impl->isNull(); }
    size_t size() const { return m_impl->size(); }

    ScriptString& operator=(const char* s)
    {
        m_impl = ScriptStringImpl::create(s);
        return *this;
    }

    ScriptString& operator+=(const String& s)
    {
        m_impl->append(s);
        return *this;
    }

    v8::Handle<v8::Value> v8StringOrNull() const
    {
        return isNull() ? v8::Handle<v8::Value>(v8::Null()) : v8::Handle<v8::Value>(m_impl->v8StringHandle());
    }

private:
    RefPtr<ScriptStringImpl> m_impl;
};

} // namespace WebCore

#endif // ScriptString_h
