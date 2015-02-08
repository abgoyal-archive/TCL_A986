

#ifndef JSDOMGlobalObject_h
#define JSDOMGlobalObject_h

#include <runtime/JSGlobalObject.h>

namespace WebCore {

    class Document;
    class Event;
    class DOMWrapperWorld;
    class JSLazyEventListener;
    class JSEventListener;
    class ScriptExecutionContext;

    typedef HashMap<const JSC::ClassInfo*, RefPtr<JSC::Structure> > JSDOMStructureMap;
    typedef HashMap<const JSC::ClassInfo*, JSC::JSObject*> JSDOMConstructorMap;

    class JSDOMGlobalObject : public JSC::JSGlobalObject {
        typedef JSC::JSGlobalObject Base;
    protected:
        struct JSDOMGlobalObjectData;

        JSDOMGlobalObject(NonNullPassRefPtr<JSC::Structure>, JSDOMGlobalObjectData*, JSC::JSObject* thisValue);

    public:
        JSDOMStructureMap& structures() { return d()->structures; }
        JSDOMConstructorMap& constructors() const { return d()->constructors; }

        virtual ScriptExecutionContext* scriptExecutionContext() const = 0;

        // Make binding code generation easier.
        JSDOMGlobalObject* globalObject() { return this; }

        void setCurrentEvent(Event*);
        Event* currentEvent() const;

        void setInjectedScript(JSObject*);
        JSObject* injectedScript() const;

        virtual void markChildren(JSC::MarkStack&);

        DOMWrapperWorld* world() { return d()->m_world.get(); }

        virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
        static const JSC::ClassInfo s_info;

    protected:
        struct JSDOMGlobalObjectData : public JSC::JSGlobalObject::JSGlobalObjectData {
            JSDOMGlobalObjectData(DOMWrapperWorld* world, Destructor destructor = destroyJSDOMGlobalObjectData)
                : JSGlobalObjectData(destructor)
                , evt(0)
                , m_world(world)
                , m_injectedScript(0)
            {
            }

            JSDOMStructureMap structures;
            JSDOMConstructorMap constructors;

            Event* evt;
            RefPtr<DOMWrapperWorld> m_world;
            JSObject* m_injectedScript;
        };

    private:
        static void destroyJSDOMGlobalObjectData(void*);

        JSDOMGlobalObjectData* d() const { return static_cast<JSDOMGlobalObjectData*>(JSC::JSVariableObject::d); }
    };

    template<class ConstructorClass>
    inline JSC::JSObject* getDOMConstructor(JSC::ExecState* exec, const JSDOMGlobalObject* globalObject)
    {
        if (JSC::JSObject* constructor = globalObject->constructors().get(&ConstructorClass::s_info))
            return constructor;
        JSC::JSObject* constructor = new (exec) ConstructorClass(exec, const_cast<JSDOMGlobalObject*>(globalObject));
        ASSERT(!globalObject->constructors().contains(&ConstructorClass::s_info));
        globalObject->constructors().set(&ConstructorClass::s_info, constructor);
        return constructor;
    }

    JSDOMGlobalObject* toJSDOMGlobalObject(Document*, JSC::ExecState*);
    JSDOMGlobalObject* toJSDOMGlobalObject(ScriptExecutionContext*, JSC::ExecState*);

    JSDOMGlobalObject* toJSDOMGlobalObject(Document*, DOMWrapperWorld*);
    JSDOMGlobalObject* toJSDOMGlobalObject(ScriptExecutionContext*, DOMWrapperWorld*);

} // namespace WebCore

#endif // JSDOMGlobalObject_h
