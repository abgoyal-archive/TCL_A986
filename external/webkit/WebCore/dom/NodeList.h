

#ifndef NodeList_h
#define NodeList_h

#include <wtf/RefCounted.h>

namespace WebCore {

    class AtomicString;
    class Node;

    class NodeList : public RefCounted<NodeList> {
    public:
        virtual ~NodeList() { }

        // DOM methods & attributes for NodeList
        virtual unsigned length() const = 0;
        virtual Node* item(unsigned index) const = 0;
        virtual Node* itemWithName(const AtomicString&) const = 0;
    };

} // namespace WebCore

#endif // NodeList_h
