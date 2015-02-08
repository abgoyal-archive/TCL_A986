

#ifndef ClassInfo_h
#define ClassInfo_h

#include "CallFrame.h"

namespace JSC {

    class HashEntry;
    struct HashTable;

    struct ClassInfo {
        /**
         * A string denoting the class name. Example: "Window".
         */
        const char* className;

        /**
         * Pointer to the class information of the base class.
         * 0L if there is none.
         */
        const ClassInfo* parentClass;
        /**
         * Static hash-table of properties.
         * For classes that can be used from multiple threads, it is accessed via a getter function that would typically return a pointer to thread-specific value.
         */
        const HashTable* propHashTable(ExecState* exec) const
        {
            if (classPropHashTableGetterFunction)
                return classPropHashTableGetterFunction(exec);
            return staticPropHashTable;
        }

        const HashTable* staticPropHashTable;
        typedef const HashTable* (*ClassPropHashTableGetterFunction)(ExecState*);
        const ClassPropHashTableGetterFunction classPropHashTableGetterFunction;
    };

} // namespace JSC

#endif // ClassInfo_h
