

#include "config.h"
#include "PropertyNameArray.h"

#include "Structure.h"
#include "StructureChain.h"

namespace JSC {

static const size_t setThreshold = 20;

void PropertyNameArray::add(UString::Rep* identifier)
{
    ASSERT(identifier == &UString::Rep::empty() || identifier->isIdentifier());

    size_t size = m_data->propertyNameVector().size();
    if (size < setThreshold) {
        for (size_t i = 0; i < size; ++i) {
            if (identifier == m_data->propertyNameVector()[i].ustring().rep())
                return;
        }
    } else {
        if (m_set.isEmpty()) {
            for (size_t i = 0; i < size; ++i)
                m_set.add(m_data->propertyNameVector()[i].ustring().rep());
        }
        if (!m_set.add(identifier).second)
            return;
    }

    addKnownUnique(identifier);
}

} // namespace JSC
