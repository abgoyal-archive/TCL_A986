

#ifndef BoundObject_h
#define BoundObject_h

#include <v8.h>
#include <wtf/Noncopyable.h>

namespace WebKit {

// BoundObject is a helper class that lets you map JavaScript method calls
// directly to C++ method calls. It should be destroyed once JS object is
// built.
class BoundObject : public Noncopyable {
public:
    BoundObject(v8::Handle<v8::Context> context, void* v8This, const char* objectName);
    virtual ~BoundObject();

    void addProtoFunction(const char* name, v8::InvocationCallback callback);
    void build();

private:
    v8::HandleScope m_handleScope;
    const char* m_objectName;
    v8::Handle<v8::Context> m_context;
    v8::Persistent<v8::FunctionTemplate> m_hostTemplate;
    void* m_v8This;
};

} // namespace WebKit

#endif
