

#ifndef JSDOMWindowShell_h
#define JSDOMWindowShell_h

#include "JSDOMWindow.h"
#include "JSDOMBinding.h"

namespace WebCore {

    class DOMWindow;
    class Frame;

    class JSDOMWindowShell : public DOMObject {
        typedef DOMObject Base;
    public:
        JSDOMWindowShell(PassRefPtr<DOMWindow>, DOMWrapperWorld* world);
        virtual ~JSDOMWindowShell();

        JSDOMWindow* window() const { return m_window; }
        void setWindow(JSDOMWindow* window)
        {
            ASSERT_ARG(window, window);
            m_window = window;
            setPrototype(window->prototype());
        }
        void setWindow(PassRefPtr<DOMWindow>);

        static const JSC::ClassInfo s_info;

        DOMWindow* impl() const;

        void* operator new(size_t);

        static PassRefPtr<JSC::Structure> createStructure(JSC::JSValue prototype) 
        {
            return JSC::Structure::create(prototype, JSC::TypeInfo(JSC::ObjectType, StructureFlags), AnonymousSlotCount); 
        }

        DOMWrapperWorld* world() { return m_world.get(); }

    private:
        static const unsigned StructureFlags = JSC::OverridesGetOwnPropertySlot | JSC::OverridesMarkChildren | JSC::OverridesGetPropertyNames | DOMObject::StructureFlags;

        virtual void markChildren(JSC::MarkStack&);
        virtual JSC::UString className() const;
        virtual bool getOwnPropertySlot(JSC::ExecState*, const JSC::Identifier& propertyName, JSC::PropertySlot&);
        virtual bool getOwnPropertyDescriptor(JSC::ExecState*, const JSC::Identifier& propertyName, JSC::PropertyDescriptor&);
        virtual void put(JSC::ExecState*, const JSC::Identifier& propertyName, JSC::JSValue, JSC::PutPropertySlot&);
        virtual void putWithAttributes(JSC::ExecState*, const JSC::Identifier& propertyName, JSC::JSValue, unsigned attributes);
        virtual bool deleteProperty(JSC::ExecState*, const JSC::Identifier& propertyName);
        virtual void getPropertyNames(JSC::ExecState*, JSC::PropertyNameArray&, JSC::EnumerationMode mode = JSC::ExcludeDontEnumProperties);
        virtual void getOwnPropertyNames(JSC::ExecState*, JSC::PropertyNameArray&, JSC::EnumerationMode mode = JSC::ExcludeDontEnumProperties);
        virtual void defineGetter(JSC::ExecState*, const JSC::Identifier& propertyName, JSC::JSObject* getterFunction, unsigned attributes);
        virtual void defineSetter(JSC::ExecState*, const JSC::Identifier& propertyName, JSC::JSObject* setterFunction, unsigned attributes);
        virtual bool defineOwnProperty(JSC::ExecState*, const JSC::Identifier& propertyName, JSC::PropertyDescriptor&, bool shouldThrow);
        virtual JSC::JSValue lookupGetter(JSC::ExecState*, const JSC::Identifier& propertyName);
        virtual JSC::JSValue lookupSetter(JSC::ExecState*, const JSC::Identifier& propertyName);
        virtual JSC::JSObject* unwrappedObject();
        virtual const JSC::ClassInfo* classInfo() const { return &s_info; }

        JSDOMWindow* m_window;
        RefPtr<DOMWrapperWorld> m_world;
    };

    JSC::JSValue toJS(JSC::ExecState*, Frame*);
    JSDOMWindowShell* toJSDOMWindowShell(Frame*, DOMWrapperWorld*);

} // namespace WebCore

#endif // JSDOMWindowShell_h
