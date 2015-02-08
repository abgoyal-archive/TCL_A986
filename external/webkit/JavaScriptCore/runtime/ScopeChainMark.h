

#ifndef ScopeChainMark_h
#define ScopeChainMark_h

#include "ScopeChain.h"

namespace JSC {

    inline void ScopeChain::markAggregate(MarkStack& markStack) const
    {
        for (ScopeChainNode* n = m_node; n; n = n->next)
            markStack.append(n->object);
    }

} // namespace JSC

#endif // ScopeChainMark_h
