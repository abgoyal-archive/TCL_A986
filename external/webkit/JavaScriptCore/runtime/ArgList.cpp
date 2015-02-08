

#include "config.h"
#include "ArgList.h"

#include "JSValue.h"
#include "JSCell.h"

using std::min;

namespace JSC {

void ArgList::getSlice(int startIndex, ArgList& result) const
{
    if (startIndex <= 0 || static_cast<unsigned>(startIndex) >= m_argCount) {
        result = ArgList(m_args, 0);
        return;
    }
    result = ArgList(m_args + startIndex, m_argCount - startIndex);
}

void MarkedArgumentBuffer::markLists(MarkStack& markStack, ListSet& markSet)
{
    ListSet::iterator end = markSet.end();
    for (ListSet::iterator it = markSet.begin(); it != end; ++it) {
        MarkedArgumentBuffer* list = *it;
        markStack.appendValues(reinterpret_cast<JSValue*>(list->m_buffer), list->m_size);
    }
}

void MarkedArgumentBuffer::slowAppend(JSValue v)
{
    // As long as our size stays within our Vector's inline 
    // capacity, all our values are allocated on the stack, and 
    // therefore don't need explicit marking. Once our size exceeds
    // our Vector's inline capacity, though, our values move to the 
    // heap, where they do need explicit marking.
    if (!m_markSet) {
        // We can only register for explicit marking once we know which heap
        // is the current one, i.e., when a non-immediate value is appended.
        if (Heap* heap = Heap::heap(v)) {
            ListSet& markSet = heap->markListSet();
            markSet.add(this);
            m_markSet = &markSet;
        }
    }

    if (m_vector.size() < m_vector.capacity()) {
        m_vector.uncheckedAppend(v);
        return;
    }

    // 4x growth would be excessive for a normal vector, but it's OK for Lists 
    // because they're short-lived.
    m_vector.reserveCapacity(m_vector.capacity() * 4);
    
    m_vector.uncheckedAppend(v);
    m_buffer = m_vector.data();
}

} // namespace JSC
