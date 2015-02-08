

#ifndef JSCustomXPathNSResolver_h
#define JSCustomXPathNSResolver_h

#if ENABLE(XPATH)

#include "XPathNSResolver.h"
#include <runtime/JSValue.h>
#include <wtf/Forward.h>
#include <wtf/RefPtr.h>

namespace JSC {
    class ExecState;
    class JSObject;
}

namespace WebCore {

    class Frame;
    class JSDOMWindow;

    class JSCustomXPathNSResolver : public XPathNSResolver {
    public:
        static PassRefPtr<JSCustomXPathNSResolver> create(JSC::ExecState*, JSC::JSValue);
        
        virtual ~JSCustomXPathNSResolver();

        virtual String lookupNamespaceURI(const String& prefix);

    private:
        JSCustomXPathNSResolver(JSC::JSObject*, JSDOMWindow*);

        // JSCustomXPathNSResolvers are always temporary, thus no need to GC protect the objects.
        JSC::JSObject* m_customResolver;
        JSDOMWindow* m_globalObject;
    };

} // namespace WebCore

#endif // ENABLE(XPATH)

#endif // JSCustomXPathNSResolver_h
