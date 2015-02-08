

#include "config.h"
#include "MarkStack.h"

namespace JSC {

size_t MarkStack::s_pageSize = 0;

void MarkStack::compact()
{
    ASSERT(s_pageSize);
    m_values.shrinkAllocation(s_pageSize);
    m_markSets.shrinkAllocation(s_pageSize);
}

}
