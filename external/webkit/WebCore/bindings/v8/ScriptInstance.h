

#ifndef ScriptInstance_h
#define ScriptInstance_h

#include <v8.h>

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class V8ScriptInstance : public RefCounted<V8ScriptInstance> {
public:
    static PassRefPtr<V8ScriptInstance> create(v8::Handle<v8::Object> instance)
    {
        return adoptRef(new V8ScriptInstance(instance));
    }
    V8ScriptInstance();
    V8ScriptInstance(v8::Handle<v8::Object>);
    ~V8ScriptInstance();
    v8::Persistent<v8::Object> instance();

private:
    void clear();
    void set(v8::Handle<v8::Object>);
    mutable v8::Persistent<v8::Object> m_instance;
};

typedef RefPtr<V8ScriptInstance> ScriptInstance;
typedef PassRefPtr<V8ScriptInstance> PassScriptInstance;

} // namespace WebCore

#endif // ScriptInstance_h
