

#ifndef CrossThreadRefCounted_h
#define CrossThreadRefCounted_h

#include <wtf/Noncopyable.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/Threading.h>

namespace WTF {

    // Used to allowing sharing data across classes and threads (like ThreadedSafeShared).
    //
    // Why not just use ThreadSafeShared?
    // ThreadSafeShared can have a significant perf impact when used in low level classes
    // (like UString) that get ref/deref'ed a lot. This class has the benefit of doing fast ref
    // counts like RefPtr whenever possible, but it has the downside that you need to copy it
    // to use it on another thread.
    //
    // Is this class threadsafe?
    // While each instance of the class is not threadsafe, the copied instance is threadsafe
    // with respect to the original and any other copies.  The underlying m_data is jointly
    // owned by the original instance and all copies.
    template<class T>
    class CrossThreadRefCounted : public Noncopyable {
    public:
        static PassRefPtr<CrossThreadRefCounted<T> > create(T* data)
        {
            return adoptRef(new CrossThreadRefCounted<T>(data, 0));
        }

        // Used to make an instance that can be used on another thread.
        PassRefPtr<CrossThreadRefCounted<T> > crossThreadCopy();

        void ref();
        void deref();
        T* release();

        bool isShared() const
        {
            return !m_refCounter.hasOneRef() || (m_threadSafeRefCounter && !m_threadSafeRefCounter->hasOneRef());
        }

    private:
        CrossThreadRefCounted(T* data, ThreadSafeSharedBase* threadedCounter)
            : m_threadSafeRefCounter(threadedCounter)
            , m_data(data)
#ifndef NDEBUG
            , m_threadId(0)
#endif
        {
        }

        ~CrossThreadRefCounted()
        {
            if (!m_threadSafeRefCounter)
                delete m_data;
        }

        void threadSafeDeref();

#ifndef NDEBUG
        bool isOwnedByCurrentThread() const { return !m_threadId || m_threadId == currentThread(); }
#endif

        RefCountedBase m_refCounter;
        ThreadSafeSharedBase* m_threadSafeRefCounter;
        T* m_data;
#ifndef NDEBUG
        ThreadIdentifier m_threadId;
#endif
    };

    template<class T>
    void CrossThreadRefCounted<T>::ref()
    {
        ASSERT(isOwnedByCurrentThread());
        m_refCounter.ref();
#ifndef NDEBUG
        // Store the threadId as soon as the ref count gets to 2.
        // The class gets created with a ref count of 1 and then passed
        // to another thread where to ref count get increased.  This
        // is a heuristic but it seems to always work and has helped
        // find some bugs.
        if (!m_threadId && m_refCounter.refCount() == 2)
            m_threadId = currentThread();
#endif
    }

    template<class T>
    void CrossThreadRefCounted<T>::deref()
    {
        ASSERT(isOwnedByCurrentThread());
        if (m_refCounter.derefBase()) {
            threadSafeDeref();
            delete this;
        } else {
#ifndef NDEBUG
            // Clear the threadId when the ref goes to 1 because it
            // is safe to be passed to another thread at this point.
            if (m_threadId && m_refCounter.refCount() == 1)
                m_threadId = 0;
#endif
        }
    }

    template<class T>
    T* CrossThreadRefCounted<T>::release()
    {
        ASSERT(!isShared());

        T* data = m_data;
        m_data = 0;
        return data;
    }

    template<class T>
    PassRefPtr<CrossThreadRefCounted<T> > CrossThreadRefCounted<T>::crossThreadCopy()
    {
        ASSERT(isOwnedByCurrentThread());
        if (m_threadSafeRefCounter)
            m_threadSafeRefCounter->ref();
        else
            m_threadSafeRefCounter = new ThreadSafeSharedBase(2);

        return adoptRef(new CrossThreadRefCounted<T>(m_data, m_threadSafeRefCounter));
    }


    template<class T>
    void CrossThreadRefCounted<T>::threadSafeDeref()
    {
        if (m_threadSafeRefCounter && m_threadSafeRefCounter->derefBase()) {
            delete m_threadSafeRefCounter;
            m_threadSafeRefCounter = 0;
        }
    }
} // namespace WTF

using WTF::CrossThreadRefCounted;

#endif // CrossThreadRefCounted_h
