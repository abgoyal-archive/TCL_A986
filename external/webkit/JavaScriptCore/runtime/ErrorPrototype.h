

#ifndef ErrorPrototype_h
#define ErrorPrototype_h

#include "ErrorInstance.h"

namespace JSC {

    class ObjectPrototype;

    class ErrorPrototype : public ErrorInstance {
    public:
        ErrorPrototype(ExecState*, NonNullPassRefPtr<Structure>, Structure* prototypeFunctionStructure);
    };

} // namespace JSC

#endif // ErrorPrototype_h
