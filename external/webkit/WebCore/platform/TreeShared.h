

#ifndef TreeShared_h
#define TreeShared_h

#include <wtf/Assertions.h>
#include <wtf/Noncopyable.h>
#ifndef NDEBUG
#include <wtf/Threading.h>
#endif

namespace WebCore {

template<class T> class TreeShared : public Noncopyable {
public:
    TreeShared(int initialRefCount = 1)
        : m_refCount(initialRefCount)
        , m_parent(0)
    {
        ASSERT(isMainThread());
#ifndef NDEBUG
        m_deletionHasBegun = false;
        m_inRemovedLastRefFunction = false;
#endif
    }
    virtual ~TreeShared()
    {
        ASSERT(isMainThread());
        ASSERT(!m_refCount);
        ASSERT(m_deletionHasBegun);
    }

    void ref()
    {
        ASSERT(isMainThread());
        ASSERT(!m_deletionHasBegun);
        ASSERT(!m_inRemovedLastRefFunction);
        ++m_refCount;
    }

    void deref()
    {
        ASSERT(isMainThread());
        ASSERT(m_refCount >= 0);
        ASSERT(!m_deletionHasBegun);
        ASSERT(!m_inRemovedLastRefFunction);
        if (--m_refCount <= 0 && !m_parent) {
#ifndef NDEBUG
            m_inRemovedLastRefFunction = true;
#endif
            removedLastRef();
        }
    }

    bool hasOneRef() const
    {
        ASSERT(!m_deletionHasBegun);
        ASSERT(!m_inRemovedLastRefFunction);
        return m_refCount == 1;
    }

    int refCount() const
    {
        return m_refCount;
    }

    void setParent(T* parent)
    { 
        ASSERT(isMainThread());
        m_parent = parent; 
    }

    T* parent() const
    {
        ASSERT(isMainThread());
        return m_parent;
    }

#ifndef NDEBUG
    bool m_deletionHasBegun;
    bool m_inRemovedLastRefFunction;
#endif

private:
    virtual void removedLastRef()
    {
#ifndef NDEBUG
        m_deletionHasBegun = true;
#endif
        delete this;
    }

    int m_refCount;
    T* m_parent;
};

}

#endif // TreeShared.h
