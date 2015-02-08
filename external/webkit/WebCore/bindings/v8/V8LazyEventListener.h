

#ifndef V8LazyEventListener_h
#define V8LazyEventListener_h

#include "PlatformString.h"
#include "V8AbstractEventListener.h"
#include <v8.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {

    class Event;
    class Frame;

    // V8LazyEventListener is a wrapper for a JavaScript code string that is compiled and evaluated when an event is fired.
    // A V8LazyEventListener is always a HTML event handler.
    class V8LazyEventListener : public V8AbstractEventListener {
    public:
        static PassRefPtr<V8LazyEventListener> create(const String& functionName, bool isSVGEvent, const String& code, const String& sourceURL, int lineNumber, int columnNumber, const WorldContextHandle& worldContext)
        {
            return adoptRef(new V8LazyEventListener(functionName, isSVGEvent, code, sourceURL, lineNumber, columnNumber, worldContext));
        }

        virtual bool isLazy() const { return true; }

    protected:
        virtual void prepareListenerObject(ScriptExecutionContext*);

    private:
        V8LazyEventListener(const String& functionName, bool isSVGEvent, const String& code, const String sourceURL, int lineNumber, int columnNumber, const WorldContextHandle& worldContext);

        virtual v8::Local<v8::Value> callListenerFunction(ScriptExecutionContext*, v8::Handle<v8::Value> jsEvent, Event*);

        String m_functionName;
        bool m_isSVGEvent;
        String m_code;
        String m_sourceURL;
        int m_lineNumber;
        int m_columnNumber;
    };

} // namespace WebCore

#endif // V8LazyEventListener_h
