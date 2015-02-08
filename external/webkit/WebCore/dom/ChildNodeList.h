

#ifndef ChildNodeList_h
#define ChildNodeList_h

#include "DynamicNodeList.h"
#include <wtf/PassRefPtr.h>

namespace WebCore {

    class ChildNodeList : public DynamicNodeList {
    public:
        static PassRefPtr<ChildNodeList> create(PassRefPtr<Node> rootNode, Caches* caches)
        {
            return adoptRef(new ChildNodeList(rootNode, caches));
        }

        virtual unsigned length() const;
        virtual Node* item(unsigned index) const;

    protected:
        ChildNodeList(PassRefPtr<Node> rootNode, Caches*);

        virtual bool nodeMatches(Element*) const;
    };

} // namespace WebCore

#endif // ChildNodeList_h
