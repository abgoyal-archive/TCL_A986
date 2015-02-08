

#ifndef ClassNodeList_h
#define ClassNodeList_h

#include "DynamicNodeList.h"
#include "SpaceSplitString.h"

namespace WebCore {

    class ClassNodeList : public DynamicNodeList {
    public:
        static PassRefPtr<ClassNodeList> create(PassRefPtr<Node> rootNode, const String& classNames, Caches* caches)
        {
            return adoptRef(new ClassNodeList(rootNode, classNames, caches));
        }

    private:
        ClassNodeList(PassRefPtr<Node> rootNode, const String& classNames, Caches*);

        virtual bool nodeMatches(Element*) const;

        SpaceSplitString m_classNames;
    };

} // namespace WebCore

#endif // ClassNodeList_h
