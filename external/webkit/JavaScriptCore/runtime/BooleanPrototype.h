

#ifndef BooleanPrototype_h
#define BooleanPrototype_h

#include "BooleanObject.h"

namespace JSC {

    class BooleanPrototype : public BooleanObject {
    public:
        BooleanPrototype(ExecState*, NonNullPassRefPtr<Structure>, Structure* prototypeFunctionStructure);
    };

} // namespace JSC

#endif // BooleanPrototype_h
