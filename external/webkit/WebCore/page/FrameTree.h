

#ifndef FrameTree_h
#define FrameTree_h

#include "AtomicString.h"

namespace WebCore {

    class Frame;

    class FrameTree : public Noncopyable {
    public:
        FrameTree(Frame* thisFrame, Frame* parentFrame) 
            : m_thisFrame(thisFrame)
            , m_parent(parentFrame)
            , m_previousSibling(0)
            , m_lastChild(0)
            , m_childCount(0)
        {
        }
        ~FrameTree();

        const AtomicString& name() const { return m_name; }
        void setName(const AtomicString&);
        void clearName();
        Frame* parent(bool checkForDisconnectedFrame = false) const;
        void setParent(Frame* parent) { m_parent = parent; }
        
        Frame* nextSibling() const { return m_nextSibling.get(); }
        Frame* previousSibling() const { return m_previousSibling; }
        Frame* firstChild() const { return m_firstChild.get(); }
        Frame* lastChild() const { return m_lastChild; }
        unsigned childCount() const { return m_childCount; }

        bool isDescendantOf(const Frame* ancestor) const;
        Frame* traverseNext(const Frame* stayWithin = 0) const;
        Frame* traverseNextWithWrap(bool) const;
        Frame* traversePreviousWithWrap(bool) const;
        
        void appendChild(PassRefPtr<Frame>);
        void detachFromParent() { m_parent = 0; }
        void removeChild(Frame*);

        Frame* child(unsigned index) const;
        Frame* child(const AtomicString& name) const;
        Frame* find(const AtomicString& name) const;

        AtomicString uniqueChildName(const AtomicString& requestedName) const;

        Frame* top(bool checkForDisconnectedFrame = false) const;

    private:
        Frame* deepLastChild() const;

        Frame* m_thisFrame;

        Frame* m_parent;
        AtomicString m_name;

        // FIXME: use ListRefPtr?
        RefPtr<Frame> m_nextSibling;
        Frame* m_previousSibling;
        RefPtr<Frame> m_firstChild;
        Frame* m_lastChild;
        unsigned m_childCount;
    };

} // namespace WebCore

#endif // FrameTree_h
