

#ifndef ScriptStringImpl_h
#define ScriptStringImpl_h

#include "OwnHandle.h"
#include "PlatformString.h"

#include <v8.h>

namespace WebCore {

// This class is used for strings that tend to be shared with JavaScript frequently.  The JSC implementation uses wtf::UString - see bindings/js/ScriptString.h
// Currently XMLHttpRequest uses a ScriptString to build up the responseText attribute.  As data arrives from the network, it is appended to the ScriptString
// via operator+= and a JavaScript readystatechange event is fired.  JavaScript can access the responseText attribute of the XMLHttpRequest object.  JavaScript
// may also query the responseXML attribute of the XMLHttpRequest object which results in the responseText attribute being coerced into a WebCore::String and
// then parsed as an XML document.
// This implementation optimizes for the common case where the responseText is built up with many calls to operator+= before the actual text is queried.
class ScriptStringImpl : public RefCounted<ScriptStringImpl> {
public:
    static PassRefPtr<ScriptStringImpl> create(const String& s)
    {
        return adoptRef(new ScriptStringImpl(s));
    }

    static PassRefPtr<ScriptStringImpl> create(const char* s)
    {
        return adoptRef(new ScriptStringImpl(s));
    }

    String toString() const;

    bool isNull() const;
    size_t size() const;

    void append(const String& s);

    v8::Handle<v8::String> v8StringHandle() { return m_handle.get(); }

private:
    ScriptStringImpl(const String& s);
    ScriptStringImpl(const char* s);

    OwnHandle<v8::String> m_handle;
};

} // namespace WebCore

#endif // ScriptStringImpl_h
