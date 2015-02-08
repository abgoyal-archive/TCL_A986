

#ifndef PropertyMapHashTable_h
#define PropertyMapHashTable_h

#include "UString.h"
#include <wtf/Vector.h>

namespace JSC {

    struct PropertyMapEntry {
        UString::Rep* key;
        unsigned offset;
        unsigned attributes;
        JSCell* specificValue;
        unsigned index;

        PropertyMapEntry(UString::Rep* key, unsigned attributes, JSCell* specificValue)
            : key(key)
            , offset(0)
            , attributes(attributes)
            , specificValue(specificValue)
            , index(0)
        {
        }

        PropertyMapEntry(UString::Rep* key, unsigned offset, unsigned attributes, JSCell* specificValue, unsigned index)
            : key(key)
            , offset(offset)
            , attributes(attributes)
            , specificValue(specificValue)
            , index(index)
        {
        }
    };

    // lastIndexUsed is an ever-increasing index used to identify the order items
    // were inserted into the property map. It's required that getEnumerablePropertyNames
    // return the properties in the order they were added for compatibility with other
    // browsers' JavaScript implementations.
    struct PropertyMapHashTable {
        unsigned sizeMask;
        unsigned size;
        unsigned keyCount;
        unsigned deletedSentinelCount;
        unsigned lastIndexUsed;
        Vector<unsigned>* deletedOffsets;
        unsigned entryIndices[1];

        PropertyMapEntry* entries()
        {
            // The entries vector comes after the indices vector.
            // The 0th item in the entries vector is not really used; it has to
            // have a 0 in its key to allow the hash table lookup to handle deleted
            // sentinels without any special-case code, but the other fields are unused.
            return reinterpret_cast<PropertyMapEntry*>(&entryIndices[size]);
        }

        static size_t allocationSize(unsigned size)
        {
            // We never let a hash table get more than half full,
            // So the number of indices we need is the size of the hash table.
            // But the number of entries is half that (plus one for the deleted sentinel).
            return sizeof(PropertyMapHashTable)
                + (size - 1) * sizeof(unsigned)
                + (1 + size / 2) * sizeof(PropertyMapEntry);
        }
    };

} // namespace JSC

#endif // PropertyMapHashTable_h
