

#ifndef WTF_HashIterators_h
#define WTF_HashIterators_h

namespace WTF {

    template<typename HashTableType, typename KeyType, typename MappedType> struct HashTableConstKeysIterator;
    template<typename HashTableType, typename KeyType, typename MappedType> struct HashTableConstValuesIterator;
    template<typename HashTableType, typename KeyType, typename MappedType> struct HashTableKeysIterator;
    template<typename HashTableType, typename KeyType, typename MappedType> struct HashTableValuesIterator;

    template<typename HashTableType, typename KeyType, typename MappedType> struct HashTableConstIteratorAdapter<HashTableType, std::pair<KeyType, MappedType> > {
    private:
        typedef std::pair<KeyType, MappedType> ValueType;
    public:
        typedef HashTableConstKeysIterator<HashTableType, KeyType, MappedType> Keys;
        typedef HashTableConstValuesIterator<HashTableType, KeyType, MappedType> Values;

        HashTableConstIteratorAdapter(const typename HashTableType::const_iterator& impl) : m_impl(impl) {}

        const ValueType* get() const { return (const ValueType*)m_impl.get(); }
        const ValueType& operator*() const { return *get(); }
        const ValueType* operator->() const { return get(); }

        HashTableConstIteratorAdapter& operator++() { ++m_impl; return *this; }
        // postfix ++ intentionally omitted

        Keys keys() { return Keys(*this); }
        Values values() { return Values(*this); }

        typename HashTableType::const_iterator m_impl;
    };

    template<typename HashTableType, typename KeyType, typename MappedType> struct HashTableIteratorAdapter<HashTableType, std::pair<KeyType, MappedType> > {
    private:
        typedef std::pair<KeyType, MappedType> ValueType;
    public:
        typedef HashTableKeysIterator<HashTableType, KeyType, MappedType> Keys;
        typedef HashTableValuesIterator<HashTableType, KeyType, MappedType> Values;

        HashTableIteratorAdapter(const typename HashTableType::iterator& impl) : m_impl(impl) {}

        ValueType* get() const { return (ValueType*)m_impl.get(); }
        ValueType& operator*() const { return *get(); }
        ValueType* operator->() const { return get(); }

        HashTableIteratorAdapter& operator++() { ++m_impl; return *this; }
        // postfix ++ intentionally omitted

        operator HashTableConstIteratorAdapter<HashTableType, ValueType>() {
            typename HashTableType::const_iterator i = m_impl;
            return i;
        }

        Keys keys() { return Keys(*this); }
        Values values() { return Values(*this); }

        typename HashTableType::iterator m_impl;
    };

    template<typename HashTableType, typename KeyType, typename MappedType> struct HashTableConstKeysIterator {
    private:
        typedef HashTableConstIteratorAdapter<HashTableType, std::pair<KeyType, MappedType> > ConstIterator;

    public:
        HashTableConstKeysIterator(const ConstIterator& impl) : m_impl(impl) {}
        
        const KeyType* get() const { return &(m_impl.get()->first); }
        const KeyType& operator*() const { return *get(); }
        const KeyType* operator->() const { return get(); }

        HashTableConstKeysIterator& operator++() { ++m_impl; return *this; }
        // postfix ++ intentionally omitted

        ConstIterator m_impl;
    };

    template<typename HashTableType, typename KeyType, typename MappedType> struct HashTableConstValuesIterator {
    private:
        typedef HashTableConstIteratorAdapter<HashTableType, std::pair<KeyType, MappedType> > ConstIterator;

    public:
        HashTableConstValuesIterator(const ConstIterator& impl) : m_impl(impl) {}
        
        const MappedType* get() const { return &(m_impl.get()->second); }
        const MappedType& operator*() const { return *get(); }
        const MappedType* operator->() const { return get(); }

        HashTableConstValuesIterator& operator++() { ++m_impl; return *this; }
        // postfix ++ intentionally omitted

        ConstIterator m_impl;
    };

    template<typename HashTableType, typename KeyType, typename MappedType> struct HashTableKeysIterator {
    private:
        typedef HashTableIteratorAdapter<HashTableType, std::pair<KeyType, MappedType> > Iterator;
        typedef HashTableConstIteratorAdapter<HashTableType, std::pair<KeyType, MappedType> > ConstIterator;

    public:
        HashTableKeysIterator(const Iterator& impl) : m_impl(impl) {}
        
        KeyType* get() const { return &(m_impl.get()->first); }
        KeyType& operator*() const { return *get(); }
        KeyType* operator->() const { return get(); }

        HashTableKeysIterator& operator++() { ++m_impl; return *this; }
        // postfix ++ intentionally omitted

        operator HashTableConstKeysIterator<HashTableType, KeyType, MappedType>() {
            ConstIterator i = m_impl;
            return i;
        }

        Iterator m_impl;
    };

    template<typename HashTableType, typename KeyType, typename MappedType> struct HashTableValuesIterator {
    private:
        typedef HashTableIteratorAdapter<HashTableType, std::pair<KeyType, MappedType> > Iterator;
        typedef HashTableConstIteratorAdapter<HashTableType, std::pair<KeyType, MappedType> > ConstIterator;

    public:
        HashTableValuesIterator(const Iterator& impl) : m_impl(impl) {}
        
        MappedType* get() const { return &(m_impl.get()->second); }
        MappedType& operator*() const { return *get(); }
        MappedType* operator->() const { return get(); }

        HashTableValuesIterator& operator++() { ++m_impl; return *this; }
        // postfix ++ intentionally omitted

        operator HashTableConstValuesIterator<HashTableType, KeyType, MappedType>() {
            ConstIterator i = m_impl;
            return i;
        }

        Iterator m_impl;
    };

    template<typename T, typename U, typename V>
        inline bool operator==(const HashTableConstKeysIterator<T, U, V>& a, const HashTableConstKeysIterator<T, U, V>& b)
    {
        return a.m_impl == b.m_impl;
    }

    template<typename T, typename U, typename V>
        inline bool operator!=(const HashTableConstKeysIterator<T, U, V>& a, const HashTableConstKeysIterator<T, U, V>& b)
    {
        return a.m_impl != b.m_impl;
    }

    template<typename T, typename U, typename V>
        inline bool operator==(const HashTableConstValuesIterator<T, U, V>& a, const HashTableConstValuesIterator<T, U, V>& b)
    {
        return a.m_impl == b.m_impl;
    }

    template<typename T, typename U, typename V>
        inline bool operator!=(const HashTableConstValuesIterator<T, U, V>& a, const HashTableConstValuesIterator<T, U, V>& b)
    {
        return a.m_impl != b.m_impl;
    }

    template<typename T, typename U, typename V>
        inline bool operator==(const HashTableKeysIterator<T, U, V>& a, const HashTableKeysIterator<T, U, V>& b)
    {
        return a.m_impl == b.m_impl;
    }

    template<typename T, typename U, typename V>
        inline bool operator!=(const HashTableKeysIterator<T, U, V>& a, const HashTableKeysIterator<T, U, V>& b)
    {
        return a.m_impl != b.m_impl;
    }

    template<typename T, typename U, typename V>
        inline bool operator==(const HashTableValuesIterator<T, U, V>& a, const HashTableValuesIterator<T, U, V>& b)
    {
        return a.m_impl == b.m_impl;
    }

    template<typename T, typename U, typename V>
        inline bool operator!=(const HashTableValuesIterator<T, U, V>& a, const HashTableValuesIterator<T, U, V>& b)
    {
        return a.m_impl != b.m_impl;
    }


} // namespace WTF

#endif // WTF_HashIterators_h
