

#ifndef NameNodeList_h
#define NameNodeList_h

#include "AtomicString.h"
#include "DynamicNodeList.h"

namespace WebCore {

    class String;

    // NodeList which lists all Nodes in a Element with a given "name" attribute
    class NameNodeList : public DynamicNodeList {
    public:
        static PassRefPtr<NameNodeList> create(PassRefPtr<Node> rootNode, const String& name, Caches* caches)
        {
            return adoptRef(new NameNodeList(rootNode, name, caches));
        }

    private:
        NameNodeList(PassRefPtr<Node> rootNode, const String& name, Caches*);

        virtual bool nodeMatches(Element*) const;

        AtomicString m_nodeName;
    };

} // namespace WebCore

#endif // NameNodeList_h
