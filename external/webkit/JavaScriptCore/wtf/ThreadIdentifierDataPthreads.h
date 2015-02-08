

#ifndef ThreadIdentifierDataPthreads_h
#define ThreadIdentifierDataPthreads_h

#include <wtf/Noncopyable.h>
#include <wtf/Threading.h>

namespace WTF {

// Holds ThreadIdentifier in the thread-specific storage and employs pthreads-specific 2-pass destruction to reliably remove
// ThreadIdentifier from threadMap. It assumes regular ThreadSpecific types don't use multiple-pass destruction.
class ThreadIdentifierData : public Noncopyable {
public:
    ~ThreadIdentifierData();

    // Creates and puts an instance of ThreadIdentifierData into thread-specific storage.
    static void initialize(ThreadIdentifier identifier);

    // Returns 0 if thread-specific storage was not initialized.
    static ThreadIdentifier identifier();

private:
    ThreadIdentifierData(ThreadIdentifier identifier)
        : m_identifier(identifier)
        , m_isDestroyedOnce(false)
    {
    }

    // This thread-specific destructor is called 2 times when thread terminates:
    // - first, when all the other thread-specific destructors are called, it simply remembers it was 'destroyed once'
    // and re-sets itself into the thread-specific slot to make Pthreads to call it again later.
    // - second, after all thread-specific destructors were invoked, it gets called again - this time, we remove the
    // ThreadIdentifier from the threadMap, completing the cleanup.
    static void destruct(void* data);

    static void initializeKeyOnceHelper();
    static void initializeKeyOnce();

    ThreadIdentifier m_identifier;
    bool m_isDestroyedOnce;
    static pthread_key_t m_key;
};

} // namespace WTF

#endif // ThreadIdentifierDataPthreads_h


