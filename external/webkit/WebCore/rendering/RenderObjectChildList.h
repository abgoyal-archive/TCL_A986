

#ifndef RenderObjectChildList_h
#define RenderObjectChildList_h

#include "RenderStyleConstants.h"

namespace WebCore {

class AtomicString;
class RenderObject;

class RenderObjectChildList {
public:
    RenderObjectChildList()
        : m_firstChild(0)
        , m_lastChild(0)
    {
    }

    RenderObject* firstChild() const { return m_firstChild; }
    RenderObject* lastChild() const { return m_lastChild; }
    
    // FIXME: Temporary while RenderBox still exists. Eventually this will just happen during insert/append/remove methods on the child list, and nobody
    // will need to manipulate firstChild or lastChild directly.
    void setFirstChild(RenderObject* child) { m_firstChild = child; }
    void setLastChild(RenderObject* child) { m_lastChild = child; }
    
    void destroyLeftoverChildren();

    RenderObject* removeChildNode(RenderObject* owner, RenderObject*, bool fullRemove = true);
    void appendChildNode(RenderObject* owner, RenderObject*, bool fullAppend = true);
    void insertChildNode(RenderObject* owner, RenderObject* child, RenderObject* before, bool fullInsert = true);

    void updateBeforeAfterContent(RenderObject* owner, PseudoId type, RenderObject* styledObject = 0);
    void invalidateCounters(RenderObject* owner, const AtomicString& identifier);

private:
    RenderObject* m_firstChild;
    RenderObject* m_lastChild;
};

} // namespace WebCore

#endif // RenderObjectChildList_h
