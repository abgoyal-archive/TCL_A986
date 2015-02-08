

#ifndef GetterSetter_h
#define GetterSetter_h

#include "JSCell.h"

#include "CallFrame.h"

namespace JSC {

    class JSObject;

    // This is an internal value object which stores getter and setter functions
    // for a property.
    class GetterSetter : public JSCell {
    public:
        GetterSetter(ExecState* exec)
            : JSCell(exec->globalData().getterSetterStructure.get())
            , m_getter(0)
            , m_setter(0)
        {
        }

        virtual void markChildren(MarkStack&);

        JSObject* getter() const { return m_getter; }
        void setGetter(JSObject* getter) { m_getter = getter; }
        JSObject* setter() const { return m_setter; }
        void setSetter(JSObject* setter) { m_setter = setter; }
        static PassRefPtr<Structure> createStructure(JSValue prototype)
        {
            return Structure::create(prototype, TypeInfo(GetterSetterType, OverridesMarkChildren), AnonymousSlotCount);
        }
    private:
        virtual bool isGetterSetter() const;

        JSObject* m_getter;
        JSObject* m_setter;  
    };

    GetterSetter* asGetterSetter(JSValue);

    inline GetterSetter* asGetterSetter(JSValue value)
    {
        ASSERT(asCell(value)->isGetterSetter());
        return static_cast<GetterSetter*>(asCell(value));
    }


} // namespace JSC

#endif // GetterSetter_h
