

#ifndef JSAPIValueWrapper_h
#define JSAPIValueWrapper_h

#include <wtf/Platform.h>

#include "JSCell.h"
#include "CallFrame.h"

namespace JSC {

    class JSAPIValueWrapper : public JSCell {
        friend JSValue jsAPIValueWrapper(ExecState*, JSValue);
    public:
        JSValue value() const { return m_value; }

        virtual bool isAPIValueWrapper() const { return true; }

        static PassRefPtr<Structure> createStructure(JSValue prototype)
        {
            return Structure::create(prototype, TypeInfo(CompoundType, OverridesMarkChildren | OverridesGetPropertyNames), AnonymousSlotCount);
        }

        
    private:
        JSAPIValueWrapper(ExecState* exec, JSValue value)
            : JSCell(exec->globalData().apiWrapperStructure.get())
            , m_value(value)
        {
            ASSERT(!value.isCell());
        }

        JSValue m_value;
    };

    inline JSValue jsAPIValueWrapper(ExecState* exec, JSValue value)
    {
        return new (exec) JSAPIValueWrapper(exec, value);
    }

} // namespace JSC

#endif // JSAPIValueWrapper_h
