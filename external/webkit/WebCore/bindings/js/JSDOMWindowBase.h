

#ifndef JSDOMWindowBase_h
#define JSDOMWindowBase_h

#include "PlatformString.h"
#include "JSDOMBinding.h"
#include <runtime/Protect.h>
#include <wtf/HashMap.h>
#include <wtf/OwnPtr.h>

namespace WebCore {

    class AtomicString;
    class DOMWindow;
    class Event;
    class Frame;
    class DOMWrapperWorld;
    class JSDOMWindow;
    class JSDOMWindowShell;
    class JSLocation;
    class JSEventListener;
    class SecurityOrigin;

    class JSDOMWindowBasePrivate;

    class JSDOMWindowBase : public JSDOMGlobalObject {
        typedef JSDOMGlobalObject Base;
    protected:
        JSDOMWindowBase(NonNullPassRefPtr<JSC::Structure>, PassRefPtr<DOMWindow>, JSDOMWindowShell*);

    public:
        void updateDocument();

        DOMWindow* impl() const { return d()->impl.get(); }
        virtual ScriptExecutionContext* scriptExecutionContext() const;

        // Called just before removing this window from the JSDOMWindowShell.
        void willRemoveFromWindowShell();

        virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
        static const JSC::ClassInfo s_info;

        virtual JSC::ExecState* globalExec();
        virtual bool supportsProfiling() const;
        virtual bool shouldInterruptScript() const;

        bool allowsAccessFrom(JSC::ExecState*) const;
        bool allowsAccessFromNoErrorMessage(JSC::ExecState*) const;
        bool allowsAccessFrom(JSC::ExecState*, String& message) const;
        void printErrorMessage(const String&) const;

        // Don't call this version of allowsAccessFrom -- it's a slightly incorrect implementation used only by WebScriptObject
        virtual bool allowsAccessFrom(const JSC::JSGlobalObject*) const;

        virtual JSC::JSObject* toThisObject(JSC::ExecState*) const;
        JSDOMWindowShell* shell() const;

        static JSC::JSGlobalData* commonJSGlobalData();

    private:
        struct JSDOMWindowBaseData : public JSDOMGlobalObjectData {
            JSDOMWindowBaseData(PassRefPtr<DOMWindow> window, JSDOMWindowShell* shell);

            RefPtr<DOMWindow> impl;
            JSDOMWindowShell* shell;
        };

        bool allowsAccessFromPrivate(const JSC::JSGlobalObject*) const;
        String crossDomainAccessErrorMessage(const JSC::JSGlobalObject*) const;
        
        static void destroyJSDOMWindowBaseData(void*);

        JSDOMWindowBaseData* d() const { return static_cast<JSDOMWindowBaseData*>(JSC::JSVariableObject::d); }
    };

    // Returns a JSDOMWindow or jsNull()
    // JSDOMGlobalObject* is ignored, accessing a window in any context will
    // use that DOMWindow's prototype chain.
    JSC::JSValue toJS(JSC::ExecState*, JSDOMGlobalObject*, DOMWindow*);
    JSC::JSValue toJS(JSC::ExecState*, DOMWindow*);

    // Returns JSDOMWindow or 0
    JSDOMWindow* toJSDOMWindow(Frame*, DOMWrapperWorld*);
    JSDOMWindow* toJSDOMWindow(JSC::JSValue);

} // namespace WebCore

#endif // JSDOMWindowBase_h
