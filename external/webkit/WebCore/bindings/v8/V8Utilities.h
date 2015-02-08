

#ifndef V8Utilities_h
#define V8Utilities_h

#include <v8.h>

namespace WebCore {

    class EventListener;
    class Frame;
    class KURL;
    class ScriptExecutionContext;
    class ScriptState;
    class String;

    // Use an array to hold dependents. It works like a ref-counted scheme. A value can be added more than once to the DOM object.
    void createHiddenDependency(v8::Handle<v8::Object>, v8::Local<v8::Value>, int cacheIndex);
    void removeHiddenDependency(v8::Handle<v8::Object>, v8::Local<v8::Value>, int cacheIndex);
    
    // Combo create/remove, for generated event-handler-setter bindings:
    void transferHiddenDependency(v8::Handle<v8::Object>, EventListener* oldValue, v8::Local<v8::Value> newValue, int cacheIndex);
    
    bool processingUserGesture();
    bool shouldAllowNavigation(Frame*);
    KURL completeURL(const String& relativeURL);
    void navigateIfAllowed(Frame*, const KURL&, bool lockHistory, bool lockBackForwardList);

    ScriptExecutionContext* getScriptExecutionContext(ScriptState*);
    inline ScriptExecutionContext* getScriptExecutionContext() {
        return getScriptExecutionContext(0);
    }

    void reportException(ScriptState*, v8::TryCatch&);

    class AllowAllocation {
    public:
        inline AllowAllocation()
        {
            m_previous = m_current;
            m_current = true;
        }

        inline ~AllowAllocation()
        {
            m_current = m_previous;
        }

        static bool m_current;

    private:
        bool m_previous;
    };

    class SafeAllocation {
     public:
      static inline v8::Local<v8::Object> newInstance(v8::Handle<v8::Function>);
      static inline v8::Local<v8::Object> newInstance(v8::Handle<v8::ObjectTemplate>);
      static inline v8::Local<v8::Object> newInstance(v8::Handle<v8::Function>, int argc, v8::Handle<v8::Value> argv[]);
    };

    v8::Local<v8::Object> SafeAllocation::newInstance(v8::Handle<v8::Function> function)
    {
        if (function.IsEmpty())
            return v8::Local<v8::Object>();
        AllowAllocation allow;
        return function->NewInstance();
    }

    v8::Local<v8::Object> SafeAllocation::newInstance(v8::Handle<v8::ObjectTemplate> objectTemplate)
    {
        if (objectTemplate.IsEmpty())
            return v8::Local<v8::Object>();
        AllowAllocation allow;
        return objectTemplate->NewInstance();
    }

    v8::Local<v8::Object> SafeAllocation::newInstance(v8::Handle<v8::Function> function, int argc, v8::Handle<v8::Value> argv[])
    {
        if (function.IsEmpty())
            return v8::Local<v8::Object>();
        AllowAllocation allow;
        return function->NewInstance(argc, argv);
    }

} // namespace WebCore

#endif // V8Utilities_h
