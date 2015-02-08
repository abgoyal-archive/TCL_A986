

#ifndef RegExpPrototype_h
#define RegExpPrototype_h

#include "JSObject.h"

namespace JSC {

    class RegExpPrototype : public JSObject {
    public:
        RegExpPrototype(ExecState*, NonNullPassRefPtr<Structure>, Structure* prototypeFunctionStructure);

        virtual const ClassInfo* classInfo() const { return &info; }
        static const ClassInfo info;
    };

} // namespace JSC

#endif // RegExpPrototype_h
