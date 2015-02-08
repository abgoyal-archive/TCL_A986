

#include "config.h"
#include "JSPropertyNameIterator.h"

#include "JSGlobalObject.h"

namespace JSC {

ASSERT_CLASS_FITS_IN_CELL(JSPropertyNameIterator);

inline JSPropertyNameIterator::JSPropertyNameIterator(ExecState* exec, PropertyNameArrayData* propertyNameArrayData, size_t numCacheableSlots)
    : JSCell(exec->globalData().propertyNameIteratorStructure.get())
    , m_cachedStructure(0)
    , m_numCacheableSlots(numCacheableSlots)
    , m_jsStringsSize(propertyNameArrayData->propertyNameVector().size())
    , m_jsStrings(new JSValue[m_jsStringsSize])
{
    PropertyNameArrayData::PropertyNameVector& propertyNameVector = propertyNameArrayData->propertyNameVector();
    for (size_t i = 0; i < m_jsStringsSize; ++i)
        m_jsStrings[i] = jsOwnedString(exec, propertyNameVector[i].ustring());
}

JSPropertyNameIterator::~JSPropertyNameIterator()
{
    if (m_cachedStructure)
        m_cachedStructure->clearEnumerationCache(this);
}

JSPropertyNameIterator* JSPropertyNameIterator::create(ExecState* exec, JSObject* o)
{
    ASSERT(!o->structure()->enumerationCache() ||
            o->structure()->enumerationCache()->cachedStructure() != o->structure() ||
            o->structure()->enumerationCache()->cachedPrototypeChain() != o->structure()->prototypeChain(exec));

    PropertyNameArray propertyNames(exec);
    o->getPropertyNames(exec, propertyNames);
    size_t numCacheableSlots = 0;
    if (!o->structure()->hasNonEnumerableProperties() && !o->structure()->hasAnonymousSlots() &&
        !o->structure()->hasGetterSetterProperties() && !o->structure()->isUncacheableDictionary() &&
        !o->structure()->typeInfo().overridesGetPropertyNames())
        numCacheableSlots = o->structure()->propertyStorageSize();

    JSPropertyNameIterator* jsPropertyNameIterator = new (exec) JSPropertyNameIterator(exec, propertyNames.data(), numCacheableSlots);

    if (o->structure()->isDictionary())
        return jsPropertyNameIterator;

    if (o->structure()->typeInfo().overridesGetPropertyNames())
        return jsPropertyNameIterator;
    
    size_t count = normalizePrototypeChain(exec, o);
    StructureChain* structureChain = o->structure()->prototypeChain(exec);
    RefPtr<Structure>* structure = structureChain->head();
    for (size_t i = 0; i < count; ++i) {
        if (structure[i]->typeInfo().overridesGetPropertyNames())
            return jsPropertyNameIterator;
    }

    jsPropertyNameIterator->setCachedPrototypeChain(structureChain);
    jsPropertyNameIterator->setCachedStructure(o->structure());
    o->structure()->setEnumerationCache(jsPropertyNameIterator);
    return jsPropertyNameIterator;
}

JSValue JSPropertyNameIterator::get(ExecState* exec, JSObject* base, size_t i)
{
    JSValue& identifier = m_jsStrings[i];
    if (m_cachedStructure == base->structure() && m_cachedPrototypeChain == base->structure()->prototypeChain(exec))
        return identifier;

    if (!base->hasProperty(exec, Identifier(exec, asString(identifier)->value(exec))))
        return JSValue();
    return identifier;
}

void JSPropertyNameIterator::markChildren(MarkStack& markStack)
{
    markStack.appendValues(m_jsStrings.get(), m_jsStringsSize, MayContainNullValues);
}

} // namespace JSC
