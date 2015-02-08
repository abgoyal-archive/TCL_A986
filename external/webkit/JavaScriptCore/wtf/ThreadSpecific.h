


#ifndef WTF_ThreadSpecific_h
#define WTF_ThreadSpecific_h

#include <wtf/Noncopyable.h>

#if USE(PTHREADS)
#include <pthread.h>
#elif PLATFORM(QT)
#include <QThreadStorage>
#elif OS(WINDOWS)
#include <windows.h>
#endif

namespace WTF {

#if !USE(PTHREADS) && !PLATFORM(QT) && OS(WINDOWS)
// ThreadSpecificThreadExit should be called each time when a thread is detached.
// This is done automatically for threads created with WTF::createThread.
void ThreadSpecificThreadExit();
#endif

template<typename T> class ThreadSpecific : public Noncopyable {
public:
    ThreadSpecific();
    T* operator->();
    operator T*();
    T& operator*();
    ~ThreadSpecific();

private:
#if !USE(PTHREADS) && !PLATFORM(QT) && OS(WINDOWS)
    friend void ThreadSpecificThreadExit();
#endif
    
    T* get();
    void set(T*);
    void static destroy(void* ptr);

#if USE(PTHREADS) || PLATFORM(QT) || OS(WINDOWS)
    struct Data : Noncopyable {
        Data(T* value, ThreadSpecific<T>* owner) : value(value), owner(owner) {}
#if PLATFORM(QT)
        ~Data() { owner->destroy(this); }
#endif

        T* value;
        ThreadSpecific<T>* owner;
#if !USE(PTHREADS) && !PLATFORM(QT)
        void (*destructor)(void*);
#endif
    };
#endif

#if ENABLE(SINGLE_THREADED)
    T* m_value;
#else
#if USE(PTHREADS)
    pthread_key_t m_key;
#elif PLATFORM(QT)
    QThreadStorage<Data*> m_key;
#elif OS(WINDOWS)
    int m_index;
#endif
#endif
};

#if ENABLE(SINGLE_THREADED)
template<typename T>
inline ThreadSpecific<T>::ThreadSpecific()
    : m_value(0)
{
}

template<typename T>
inline ThreadSpecific<T>::~ThreadSpecific()
{
}

template<typename T>
inline T* ThreadSpecific<T>::get()
{
    return m_value;
}

template<typename T>
inline void ThreadSpecific<T>::set(T* ptr)
{
    ASSERT(!get());
    m_value = ptr;
}
#else
#if USE(PTHREADS)
template<typename T>
inline ThreadSpecific<T>::ThreadSpecific()
{
    int error = pthread_key_create(&m_key, destroy);
    if (error)
        CRASH();
}

template<typename T>
inline ThreadSpecific<T>::~ThreadSpecific()
{
    pthread_key_delete(m_key); // Does not invoke destructor functions.
}

template<typename T>
inline T* ThreadSpecific<T>::get()
{
    Data* data = static_cast<Data*>(pthread_getspecific(m_key));
    return data ? data->value : 0;
}

template<typename T>
inline void ThreadSpecific<T>::set(T* ptr)
{
    ASSERT(!get());
    pthread_setspecific(m_key, new Data(ptr, this));
}

#elif PLATFORM(QT)

template<typename T>
inline ThreadSpecific<T>::ThreadSpecific()
{
}

template<typename T>
inline ThreadSpecific<T>::~ThreadSpecific()
{
    // Does not invoke destructor functions. QThreadStorage will do it
}

template<typename T>
inline T* ThreadSpecific<T>::get()
{
    Data* data = static_cast<Data*>(m_key.localData());
    return data ? data->value : 0;
}

template<typename T>
inline void ThreadSpecific<T>::set(T* ptr)
{
    ASSERT(!get());
    Data* data = new Data(ptr, this);
    m_key.setLocalData(data);
}

#elif OS(WINDOWS)

// TLS_OUT_OF_INDEXES is not defined on WinCE.
#ifndef TLS_OUT_OF_INDEXES
#define TLS_OUT_OF_INDEXES 0xffffffff
#endif

// The maximum number of TLS keys that can be created. For simplification, we assume that:
// 1) Once the instance of ThreadSpecific<> is created, it will not be destructed until the program dies.
// 2) We do not need to hold many instances of ThreadSpecific<> data. This fixed number should be far enough.
const int kMaxTlsKeySize = 256;

long& tlsKeyCount();
DWORD* tlsKeys();

template<typename T>
inline ThreadSpecific<T>::ThreadSpecific()
    : m_index(-1)
{
    DWORD tlsKey = TlsAlloc();
    if (tlsKey == TLS_OUT_OF_INDEXES)
        CRASH();

    m_index = InterlockedIncrement(&tlsKeyCount()) - 1;
    if (m_index >= kMaxTlsKeySize)
        CRASH();
    tlsKeys()[m_index] = tlsKey;
}

template<typename T>
inline ThreadSpecific<T>::~ThreadSpecific()
{
    // Does not invoke destructor functions. They will be called from ThreadSpecificThreadExit when the thread is detached.
    TlsFree(tlsKeys()[m_index]);
}

template<typename T>
inline T* ThreadSpecific<T>::get()
{
    Data* data = static_cast<Data*>(TlsGetValue(tlsKeys()[m_index]));
    return data ? data->value : 0;
}

template<typename T>
inline void ThreadSpecific<T>::set(T* ptr)
{
    ASSERT(!get());
    Data* data = new Data(ptr, this);
    data->destructor = &ThreadSpecific<T>::destroy;
    TlsSetValue(tlsKeys()[m_index], data);
}

#else
#error ThreadSpecific is not implemented for this platform.
#endif
#endif

template<typename T>
inline void ThreadSpecific<T>::destroy(void* ptr)
{
#if !ENABLE(SINGLE_THREADED)
    Data* data = static_cast<Data*>(ptr);

#if USE(PTHREADS)
    // We want get() to keep working while data destructor works, because it can be called indirectly by the destructor.
    // Some pthreads implementations zero out the pointer before calling destroy(), so we temporarily reset it.
    pthread_setspecific(data->owner->m_key, ptr);
#endif
#if PLATFORM(QT)
    // See comment as above
    data->owner->m_key.setLocalData(data);
#endif

    data->value->~T();
    fastFree(data->value);

#if USE(PTHREADS)
    pthread_setspecific(data->owner->m_key, 0);
#elif PLATFORM(QT)
    // Do nothing here
#elif OS(WINDOWS)
    TlsSetValue(tlsKeys()[data->owner->m_index], 0);
#else
#error ThreadSpecific is not implemented for this platform.
#endif

#if !PLATFORM(QT)
    delete data;
#endif
#endif
}

template<typename T>
inline ThreadSpecific<T>::operator T*()
{
    T* ptr = static_cast<T*>(get());
    if (!ptr) {
        // Set up thread-specific value's memory pointer before invoking constructor, in case any function it calls
        // needs to access the value, to avoid recursion.
        ptr = static_cast<T*>(fastMalloc(sizeof(T)));
        set(ptr);
        new (ptr) T;
    }
    return ptr;
}

template<typename T>
inline T* ThreadSpecific<T>::operator->()
{
    return operator T*();
}

template<typename T>
inline T& ThreadSpecific<T>::operator*()
{
    return *operator T*();
}

}

#endif
