

#ifndef ScriptArray_h
#define ScriptArray_h

#include "ScriptObject.h"
#include "ScriptState.h"

#include <runtime/JSArray.h>

namespace WebCore {

    class ScriptArray : public ScriptObject {
    public:
        ScriptArray(ScriptState*, JSC::JSArray*);
        ScriptArray() {}
        JSC::JSArray* jsArray() const { return asArray(jsValue()); }

        bool set(unsigned index, const ScriptObject&);
        bool set(unsigned index, const String&);
        bool set(unsigned index, double);
        bool set(unsigned index, long long);
        bool set(unsigned index, int);
        bool set(unsigned index, bool);
        unsigned length();

        static ScriptArray createNew(ScriptState*);
    };
}

#endif // ScriptArray_h
