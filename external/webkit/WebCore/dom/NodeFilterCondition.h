

#ifndef NodeFilterCondition_h
#define NodeFilterCondition_h

#include "ScriptState.h"
#include <wtf/RefCounted.h>

namespace JSC {
    class MarkStack;
}

namespace WebCore {

    class Node;

    class NodeFilterCondition : public RefCounted<NodeFilterCondition> {
    public:
        virtual ~NodeFilterCondition() { }
        virtual short acceptNode(ScriptState*, Node*) const = 0;
        virtual void markAggregate(JSC::MarkStack&) { }
    };

} // namespace WebCore

#endif // NodeFilterCondition_h
