

#ifndef TagNodeList_h
#define TagNodeList_h

#include "AtomicString.h"
#include "DynamicNodeList.h"

namespace WebCore {

    // NodeList that limits to a particular tag.
    class TagNodeList : public DynamicNodeList {
    public:
        static PassRefPtr<TagNodeList> create(PassRefPtr<Node> rootNode, const AtomicString& namespaceURI, const AtomicString& localName, DynamicNodeList::Caches* caches)
        {
            return adoptRef(new TagNodeList(rootNode, namespaceURI, localName, caches));
        }

    private:
        TagNodeList(PassRefPtr<Node> rootNode, const AtomicString& namespaceURI, const AtomicString& localName, DynamicNodeList::Caches* caches);

        virtual bool nodeMatches(Element*) const;

        AtomicString m_namespaceURI;
        AtomicString m_localName;
    };

} // namespace WebCore

#endif // TagNodeList_h
