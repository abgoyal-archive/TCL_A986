

#ifndef ObjCNodeFilterCondition_h
#define ObjCNodeFilterCondition_h

#include "NodeFilterCondition.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RetainPtr.h>

@protocol DOMNodeFilter;

namespace WebCore {
    class Node;

    class ObjCNodeFilterCondition : public NodeFilterCondition {
    public:
        static PassRefPtr<ObjCNodeFilterCondition> create(id <DOMNodeFilter> filter)
        {
            return adoptRef(new ObjCNodeFilterCondition(filter));
        }

        virtual short acceptNode(JSC::ExecState*, Node*) const;

    private:
        ObjCNodeFilterCondition(id <DOMNodeFilter> filter)
            : m_filter(filter)
        {
        }

        RetainPtr<id <DOMNodeFilter> > m_filter;
    };

} // namespace WebCore

#endif
