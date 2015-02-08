

#ifndef TreeWalker_h
#define TreeWalker_h

#include "NodeFilter.h"
#include "Traversal.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

    typedef int ExceptionCode;

    class TreeWalker : public RefCounted<TreeWalker>, public Traversal {
    public:
        static PassRefPtr<TreeWalker> create(PassRefPtr<Node> rootNode, unsigned whatToShow, PassRefPtr<NodeFilter> filter, bool expandEntityReferences)
        {
            return adoptRef(new TreeWalker(rootNode, whatToShow, filter, expandEntityReferences));
        }                            

        Node* currentNode() const { return m_current.get(); }
        void setCurrentNode(PassRefPtr<Node>, ExceptionCode&);

        Node* parentNode(ScriptState*);
        Node* firstChild(ScriptState*);
        Node* lastChild(ScriptState*);
        Node* previousSibling(ScriptState*);
        Node* nextSibling(ScriptState*);
        Node* previousNode(ScriptState*);
        Node* nextNode(ScriptState*);

        // Do not call these functions. They are just scaffolding to support the Objective-C bindings.
        // They operate in the main thread normal world, and they swallow JS exceptions.
        Node* parentNode() { return parentNode(scriptStateFromNode(mainThreadNormalWorld(), m_current.get())); }
        Node* firstChild() { return firstChild(scriptStateFromNode(mainThreadNormalWorld(), m_current.get())); }
        Node* lastChild() { return lastChild(scriptStateFromNode(mainThreadNormalWorld(), m_current.get())); }
        Node* previousSibling() { return previousSibling(scriptStateFromNode(mainThreadNormalWorld(), m_current.get())); }
        Node* nextSibling() { return nextSibling(scriptStateFromNode(mainThreadNormalWorld(), m_current.get())); }
        Node* previousNode() { return previousNode(scriptStateFromNode(mainThreadNormalWorld(), m_current.get())); }
        Node* nextNode() { return nextNode(scriptStateFromNode(mainThreadNormalWorld(), m_current.get())); }

    private:
        TreeWalker(PassRefPtr<Node>, unsigned whatToShow, PassRefPtr<NodeFilter>, bool expandEntityReferences);
        
        Node* setCurrent(PassRefPtr<Node>);

        RefPtr<Node> m_current;
    };

} // namespace WebCore

#endif // TreeWalker_h
