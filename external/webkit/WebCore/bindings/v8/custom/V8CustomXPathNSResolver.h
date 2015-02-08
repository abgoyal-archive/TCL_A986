


#ifndef V8CustomXPathNSResolver_h
#define V8CustomXPathNSResolver_h

#if ENABLE(XPATH)

#include "XPathNSResolver.h"
#include <v8.h>
#include <wtf/Forward.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class String;
class V8Proxy;

// V8CustomXPathNSResolver does not create a persistent handle to the
// given resolver object.  So the lifetime of V8CustomXPathNSResolver
// must not exceed the lifetime of the passed handle.
class V8CustomXPathNSResolver : public XPathNSResolver {
public:
    static PassRefPtr<V8CustomXPathNSResolver> create(V8Proxy* proxy, v8::Handle<v8::Object> resolver);

    virtual ~V8CustomXPathNSResolver();
    virtual String lookupNamespaceURI(const String& prefix);

private:
    V8CustomXPathNSResolver(V8Proxy* proxy, v8::Handle<v8::Object> resolver);

    V8Proxy* m_proxy;
    v8::Handle<v8::Object> m_resolver;  // Handle to resolver object.
};

} // namespace WebCore

#endif  // ENABLE(XPATH)

#endif  // V8CustomXPathNSResolver_h
