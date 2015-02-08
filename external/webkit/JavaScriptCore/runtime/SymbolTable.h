

#ifndef SymbolTable_h
#define SymbolTable_h

#include "JSObject.h"
#include "UString.h"
#include <wtf/AlwaysInline.h>

namespace JSC {

    static ALWAYS_INLINE int missingSymbolMarker() { return std::numeric_limits<int>::max(); }

    // The bit twiddling in this class assumes that every register index is a
    // reasonably small positive or negative number, and therefore has its high
    // four bits all set or all unset.

    struct SymbolTableEntry {
        SymbolTableEntry()
            : m_bits(0)
        {
        }

        SymbolTableEntry(int index)
        {
            ASSERT(isValidIndex(index));
            pack(index, false, false);
        }

        SymbolTableEntry(int index, unsigned attributes)
        {
            ASSERT(isValidIndex(index));
            pack(index, attributes & ReadOnly, attributes & DontEnum);
        }
        
        bool isNull() const
        {
            return !m_bits;
        }

        int getIndex() const
        {
            return m_bits >> FlagBits;
        }

        unsigned getAttributes() const
        {
            unsigned attributes = 0;
            if (m_bits & ReadOnlyFlag)
                attributes |= ReadOnly;
            if (m_bits & DontEnumFlag)
                attributes |= DontEnum;
            return attributes;
        }

        void setAttributes(unsigned attributes)
        {
            pack(getIndex(), attributes & ReadOnly, attributes & DontEnum);
        }

        bool isReadOnly() const
        {
            return m_bits & ReadOnlyFlag;
        }

    private:
        static const unsigned ReadOnlyFlag = 0x1;
        static const unsigned DontEnumFlag = 0x2;
        static const unsigned NotNullFlag = 0x4;
        static const unsigned FlagBits = 3;

        void pack(int index, bool readOnly, bool dontEnum)
        {
            m_bits = (index << FlagBits) | NotNullFlag;
            if (readOnly)
                m_bits |= ReadOnlyFlag;
            if (dontEnum)
                m_bits |= DontEnumFlag;
        }
        
        bool isValidIndex(int index)
        {
            return ((index << FlagBits) >> FlagBits) == index;
        }

        int m_bits;
    };

    struct SymbolTableIndexHashTraits {
        typedef SymbolTableEntry TraitType;
        static SymbolTableEntry emptyValue() { return SymbolTableEntry(); }
        static const bool emptyValueIsZero = true;
        static const bool needsDestruction = false;
    };

    typedef HashMap<RefPtr<UString::Rep>, SymbolTableEntry, IdentifierRepHash, HashTraits<RefPtr<UString::Rep> >, SymbolTableIndexHashTraits> SymbolTable;

    class SharedSymbolTable : public SymbolTable, public RefCounted<SharedSymbolTable>
    {
    };
    
} // namespace JSC

#endif // SymbolTable_h
