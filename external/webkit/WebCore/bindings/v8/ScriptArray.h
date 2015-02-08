

#ifndef ScriptArray_h
#define ScriptArray_h

#include "ScriptObject.h"

#include <v8.h>

namespace WebCore {
    class ScriptState;

    class ScriptArray : public ScriptObject {
    public:
        ScriptArray(ScriptState* scriptState, v8::Handle<v8::Array>);
        virtual ~ScriptArray() {}

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
