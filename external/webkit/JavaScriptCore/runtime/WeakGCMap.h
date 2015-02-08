

#ifndef WeakGCMap_h
#define WeakGCMap_h

#include "Collector.h"
#include <wtf/HashMap.h>

namespace JSC {

class JSCell;

// A HashMap whose get() function returns emptyValue() for cells awaiting destruction.
template<typename KeyType, typename MappedType>
class WeakGCMap : public FastAllocBase {
    /*
    Invariants:
        * A value enters the WeakGCMap marked. (Guaranteed by set().)
        * A value that becomes unmarked leaves the WeakGCMap before being recycled. (Guaranteed by the value's destructor removing it from the WeakGCMap.)
        * A value that becomes unmarked leaves the WeakGCMap before becoming marked again. (Guaranteed by all destructors running before the mark phase begins.)
        * During the mark phase, all values in the WeakGCMap are valid. (Guaranteed by all destructors running before the mark phase begins.)
    */

public:
    typedef typename HashMap<KeyType, MappedType>::iterator iterator;
    typedef typename HashMap<KeyType, MappedType>::const_iterator const_iterator;
    
    bool isEmpty() { return m_map.isEmpty(); }

    MappedType get(const KeyType& key) const;
    pair<iterator, bool> set(const KeyType&, const MappedType&); 
    MappedType take(const KeyType& key);

    // These unchecked functions provide access to a value even if the value's
    // mark bit is not set. This is used, among other things, to retrieve values
    // during the GC mark phase, which begins by clearing all mark bits.

    MappedType uncheckedGet(const KeyType& key) const { return m_map.get(key); }
    bool uncheckedRemove(const KeyType&, const MappedType&);

    iterator uncheckedBegin() { return m_map.begin(); }
    iterator uncheckedEnd() { return m_map.end(); }

    const_iterator uncheckedBegin() const { return m_map.begin(); }
    const_iterator uncheckedEnd() const { return m_map.end(); }

private:
    HashMap<KeyType, MappedType> m_map;
};

template<typename KeyType, typename MappedType>
inline MappedType WeakGCMap<KeyType, MappedType>::get(const KeyType& key) const
{
    MappedType result = m_map.get(key);
    if (result == HashTraits<MappedType>::emptyValue())
        return result;
    if (!Heap::isCellMarked(result))
        return HashTraits<MappedType>::emptyValue();
    return result;
}

template<typename KeyType, typename MappedType>
MappedType WeakGCMap<KeyType, MappedType>::take(const KeyType& key)
{
    MappedType result = m_map.take(key);
    if (result == HashTraits<MappedType>::emptyValue())
        return result;
    if (!Heap::isCellMarked(result))
        return HashTraits<MappedType>::emptyValue();
    return result;
}

template<typename KeyType, typename MappedType>
pair<typename HashMap<KeyType, MappedType>::iterator, bool> WeakGCMap<KeyType, MappedType>::set(const KeyType& key, const MappedType& value)
{
    Heap::markCell(value); // If value is newly allocated, it's not marked, so mark it now.
    pair<iterator, bool> result = m_map.add(key, value);
    if (!result.second) { // pre-existing entry
        result.second = !Heap::isCellMarked(result.first->second);
        result.first->second = value;
    }
    return result;
}

template<typename KeyType, typename MappedType>
bool WeakGCMap<KeyType, MappedType>::uncheckedRemove(const KeyType& key, const MappedType& value)
{
    iterator it = m_map.find(key);
    if (it == m_map.end())
        return false;
    if (it->second != value)
        return false;
    m_map.remove(it);
    return true;
}

} // namespace JSC

#endif // WeakGCMap_h
