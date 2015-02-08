

#ifndef ObjectPrototype_h
#define ObjectPrototype_h

#include "JSObject.h"

namespace JSC {

    class ObjectPrototype : public JSObject {
    public:
        ObjectPrototype(ExecState*, NonNullPassRefPtr<Structure>, Structure* prototypeFunctionStructure);

    private:
        virtual void put(ExecState*, const Identifier&, JSValue, PutPropertySlot&);
        virtual bool getOwnPropertySlot(ExecState*, unsigned propertyName, PropertySlot&);

        bool m_hasNoPropertiesWithUInt32Names;
    };

    JSValue JSC_HOST_CALL objectProtoFuncToString(ExecState*, JSObject*, JSValue, const ArgList&);

} // namespace JSC

#endif // ObjectPrototype_h
