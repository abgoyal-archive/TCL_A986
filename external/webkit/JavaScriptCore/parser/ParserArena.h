

#ifndef ParserArena_h
#define ParserArena_h

#include "Identifier.h"
#include <wtf/SegmentedVector.h>

namespace JSC {

    class ParserArenaDeletable;
    class ParserArenaRefCounted;

    class IdentifierArena : public FastAllocBase {
    public:
        ALWAYS_INLINE const Identifier& makeIdentifier(JSGlobalData*, const UChar* characters, size_t length);
        const Identifier& makeNumericIdentifier(JSGlobalData*, double number);

        void clear() { m_identifiers.clear(); }
        bool isEmpty() const { return m_identifiers.isEmpty(); }

    private:
        typedef SegmentedVector<Identifier, 64> IdentifierVector;
        IdentifierVector m_identifiers;
    };

    ALWAYS_INLINE const Identifier& IdentifierArena::makeIdentifier(JSGlobalData* globalData, const UChar* characters, size_t length)
    {
        m_identifiers.append(Identifier(globalData, characters, length));
        return m_identifiers.last();
    }

    inline const Identifier& IdentifierArena::makeNumericIdentifier(JSGlobalData* globalData, double number)
    {
        m_identifiers.append(Identifier(globalData, UString::from(number)));
        return m_identifiers.last();
    }

    class ParserArena : Noncopyable {
    public:
        ParserArena();
        ~ParserArena();

        void swap(ParserArena& otherArena)
        {
            std::swap(m_freeableMemory, otherArena.m_freeableMemory);
            std::swap(m_freeablePoolEnd, otherArena.m_freeablePoolEnd);
            m_identifierArena.swap(otherArena.m_identifierArena);
            m_freeablePools.swap(otherArena.m_freeablePools);
            m_deletableObjects.swap(otherArena.m_deletableObjects);
            m_refCountedObjects.swap(otherArena.m_refCountedObjects);
        }

        void* allocateFreeable(size_t size)
        {
            ASSERT(size);
            ASSERT(size <= freeablePoolSize);
            size_t alignedSize = alignSize(size);
            ASSERT(alignedSize <= freeablePoolSize);
            if (UNLIKELY(static_cast<size_t>(m_freeablePoolEnd - m_freeableMemory) < alignedSize))
                allocateFreeablePool();
            void* block = m_freeableMemory;
            m_freeableMemory += alignedSize;
            return block;
        }

        void* allocateDeletable(size_t size)
        {
            ParserArenaDeletable* deletable = static_cast<ParserArenaDeletable*>(fastMalloc(size));
            m_deletableObjects.append(deletable);
            return deletable;
        }

        void derefWithArena(PassRefPtr<ParserArenaRefCounted>);
        bool contains(ParserArenaRefCounted*) const;
        ParserArenaRefCounted* last() const;
        void removeLast();

        bool isEmpty() const;
        void reset();

        IdentifierArena& identifierArena() { return *m_identifierArena; }

    private:
        static const size_t freeablePoolSize = 8000;

        static size_t alignSize(size_t size)
        {
            return (size + sizeof(WTF::AllocAlignmentInteger) - 1) & ~(sizeof(WTF::AllocAlignmentInteger) - 1);
        }

        void* freeablePool();
        void allocateFreeablePool();
        void deallocateObjects();

        char* m_freeableMemory;
        char* m_freeablePoolEnd;

        OwnPtr<IdentifierArena> m_identifierArena;
        Vector<void*> m_freeablePools;
        Vector<ParserArenaDeletable*> m_deletableObjects;
        Vector<RefPtr<ParserArenaRefCounted> > m_refCountedObjects;
    };

}

#endif
