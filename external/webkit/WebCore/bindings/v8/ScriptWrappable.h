

#ifndef ScriptWrappable_h
#define ScriptWrappable_h

#include <v8.h>

namespace WebCore {

class ScriptWrappable {
public:
    ScriptWrappable() : m_wrapper(0) { }

    v8::Persistent<v8::Object>* wrapper() const
    {
        return m_wrapper;
    }

    void setWrapper(v8::Persistent<v8::Object>* wrapper)
    {
        ASSERT(wrapper);
        m_wrapper = wrapper;
    }

    void clearWrapper() { m_wrapper = 0; }

private:
    v8::Persistent<v8::Object>* m_wrapper;
};

} // namespace WebCore

#endif // ScriptWrappable_h
