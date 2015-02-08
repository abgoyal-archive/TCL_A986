

#ifndef JSLazyEventListener_h
#define JSLazyEventListener_h

#include "JSEventListener.h"
#include "PlatformString.h"

namespace WebCore {

    class Node;

    class JSLazyEventListener : public JSEventListener {
    public:
        static PassRefPtr<JSLazyEventListener> create(const String& functionName, const String& eventParameterName, const String& code, Node* node, const String& sourceURL, int lineNumber, JSC::JSObject* wrapper, DOMWrapperWorld* isolatedWorld)
        {
            return adoptRef(new JSLazyEventListener(functionName, eventParameterName, code, node, sourceURL, lineNumber, wrapper, isolatedWorld));
        }
        virtual ~JSLazyEventListener();

    private:
        JSLazyEventListener(const String& functionName, const String& eventParameterName, const String& code, Node*, const String& sourceURL, int lineNumber, JSC::JSObject* wrapper, DOMWrapperWorld* isolatedWorld);

        virtual JSC::JSObject* initializeJSFunction(ScriptExecutionContext*) const;
        virtual bool wasCreatedFromMarkup() const { return true; }

        mutable String m_functionName;
        mutable String m_eventParameterName;
        mutable String m_code;
        mutable String m_sourceURL;
        int m_lineNumber;
        Node* m_originalNode;
    };

} // namespace WebCore

#endif // JSEventListener_h
