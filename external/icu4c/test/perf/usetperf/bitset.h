#ifndef __BITSET_H__
#define __BITSET_H__

#include "unicode/utypes.h"

class BitSet {

    uint32_t len;
    int32_t* data;

    void ensureCapacity(uint32_t minLen);

public:

    BitSet();
    ~BitSet();

    UBool get(int32_t bitIndex) const;

    void set(int32_t bitIndex);

    // Non-java
    void clearAll();

    // TODO add other methods as needed.
};

#endif
