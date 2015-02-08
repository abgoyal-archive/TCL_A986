

#ifndef DynamicNodeList_h
#define DynamicNodeList_h

#include "NodeList.h"
#include <wtf/RefCounted.h>
#include <wtf/Forward.h>
#include <wtf/RefPtr.h>

namespace WebCore {

    class AtomicString;
    class Element;
    class Node;

    class DynamicNodeList : public NodeList {
    public:
        struct Caches : RefCounted<Caches> {
            static PassRefPtr<Caches> create();
            void reset();

            unsigned cachedLength;
            Node* lastItem;
            unsigned lastItemOffset;
            bool isLengthCacheValid : 1;
            bool isItemCacheValid : 1;
        protected:
            Caches();
        };

        virtual ~DynamicNodeList();

        bool hasOwnCaches() const { return m_ownsCaches; }

        // DOM methods & attributes for NodeList
        virtual unsigned length() const;
        virtual Node* item(unsigned index) const;
        virtual Node* itemWithName(const AtomicString&) const;

        // Other methods (not part of DOM)
        void invalidateCache();

    protected:
        DynamicNodeList(PassRefPtr<Node> rootNode);
        DynamicNodeList(PassRefPtr<Node> rootNode, Caches*);

        virtual bool nodeMatches(Element*) const = 0;

        RefPtr<Node> m_rootNode;
        mutable RefPtr<Caches> m_caches;
        bool m_ownsCaches;

    private:
        Node* itemForwardsFromCurrent(Node* start, unsigned offset, int remainingOffset) const;
        Node* itemBackwardsFromCurrent(Node* start, unsigned offset, int remainingOffset) const;
    };

} // namespace WebCore

#endif // DynamicNodeList_h
