

#ifndef JSPropertyNameIterator_h
#define JSPropertyNameIterator_h

#include "JSObject.h"
#include "JSString.h"
#include "Operations.h"
#include "PropertyNameArray.h"

namespace JSC {

    class Identifier;
    class JSObject;

    class JSPropertyNameIterator : public JSCell {
        friend class JIT;

    public:
        static JSPropertyNameIterator* create(ExecState*, JSObject*);
        
        static PassRefPtr<Structure> createStructure(JSValue prototype)
        {
            return Structure::create(prototype, TypeInfo(CompoundType, OverridesMarkChildren), AnonymousSlotCount);
        }
        
        virtual ~JSPropertyNameIterator();

        virtual bool isPropertyNameIterator() const { return true; }

        virtual void markChildren(MarkStack&);

        bool getOffset(size_t i, int& offset)
        {
            if (i >= m_numCacheableSlots)
                return false;
            offset = i;
            return true;
        }

        JSValue get(ExecState*, JSObject*, size_t i);
        size_t size() { return m_jsStringsSize; }

        void setCachedStructure(Structure* structure) { m_cachedStructure = structure; }
        Structure* cachedStructure() { return m_cachedStructure; }

        void setCachedPrototypeChain(NonNullPassRefPtr<StructureChain> cachedPrototypeChain) { m_cachedPrototypeChain = cachedPrototypeChain; }
        StructureChain* cachedPrototypeChain() { return m_cachedPrototypeChain.get(); }

    private:
        JSPropertyNameIterator(ExecState*, PropertyNameArrayData* propertyNameArrayData, size_t numCacheableSlot);

        Structure* m_cachedStructure;
        RefPtr<StructureChain> m_cachedPrototypeChain;
        uint32_t m_numCacheableSlots;
        uint32_t m_jsStringsSize;
        OwnArrayPtr<JSValue> m_jsStrings;
    };

    inline void Structure::setEnumerationCache(JSPropertyNameIterator* enumerationCache)
    {
        ASSERT(!isDictionary());
        m_enumerationCache = enumerationCache;
    }

    inline void Structure::clearEnumerationCache(JSPropertyNameIterator* enumerationCache)
    {
        m_enumerationCache.clear(enumerationCache);
    }

    inline JSPropertyNameIterator* Structure::enumerationCache()
    {
        return m_enumerationCache.get();
    }

} // namespace JSC

#endif // JSPropertyNameIterator_h
