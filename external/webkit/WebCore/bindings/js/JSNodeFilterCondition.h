

#ifndef JSNodeFilterCondition_h
#define JSNodeFilterCondition_h

#include "NodeFilterCondition.h"
#include <runtime/JSValue.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {

    class Node;

    class JSNodeFilterCondition : public NodeFilterCondition {
    public:
        static PassRefPtr<JSNodeFilterCondition> create(JSC::JSValue filter)
        {
            return adoptRef(new JSNodeFilterCondition(filter));
        }

    private:
        JSNodeFilterCondition(JSC::JSValue filter);

        virtual short acceptNode(ScriptState*, Node*) const;
        virtual void markAggregate(JSC::MarkStack&);

        mutable JSC::JSValue m_filter;
    };

} // namespace WebCore

#endif // JSNodeFilterCondition_h
