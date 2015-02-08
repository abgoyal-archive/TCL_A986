

#include "config.h"
#include "StructureChain.h"

#include "JSObject.h"
#include "Structure.h"
#include <wtf/RefPtr.h>

namespace JSC {

StructureChain::StructureChain(Structure* head)
{
    size_t size = 0;
    for (Structure* current = head; current; current = current->storedPrototype().isNull() ? 0 : asObject(current->storedPrototype())->structure())
        ++size;
    
    m_vector.set(new RefPtr<Structure>[size + 1]);

    size_t i = 0;
    for (Structure* current = head; current; current = current->storedPrototype().isNull() ? 0 : asObject(current->storedPrototype())->structure())
        m_vector[i++] = current;
    m_vector[i] = 0;
}

} // namespace JSC
