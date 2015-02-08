

#ifndef NativeErrorPrototype_h
#define NativeErrorPrototype_h

#include "JSObject.h"

namespace JSC {

    class NativeErrorPrototype : public JSObject {
    public:
        NativeErrorPrototype(ExecState*, NonNullPassRefPtr<Structure>, const UString& name, const UString& message);
    };

} // namespace JSC

#endif // NativeErrorPrototype_h
