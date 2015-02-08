

#ifndef GOwnPtr_h
#define GOwnPtr_h

#include <algorithm>
#include <wtf/Assertions.h>
#include <wtf/Noncopyable.h>

// Forward delcarations at this point avoid the need to include GLib includes
// in WTF headers.
typedef struct _GError GError;
typedef struct _GList GList;
typedef struct _GCond GCond;
typedef struct _GMutex GMutex;
typedef struct _GPatternSpec GPatternSpec;
typedef struct _GDir GDir;
typedef struct _GHashTable GHashTable;
extern "C" void g_free(void*);

namespace WTF {

template <typename T> inline void freeOwnedGPtr(T* ptr);
template<> void freeOwnedGPtr<GError>(GError*);
template<> void freeOwnedGPtr<GList>(GList*);
template<> void freeOwnedGPtr<GCond>(GCond*);
template<> void freeOwnedGPtr<GMutex>(GMutex*);
template<> void freeOwnedGPtr<GPatternSpec>(GPatternSpec*);
template<> void freeOwnedGPtr<GDir>(GDir*);
template<> void freeOwnedGPtr<GHashTable>(GHashTable*);

template <typename T> class GOwnPtr : public Noncopyable {
public:
    explicit GOwnPtr(T* ptr = 0) : m_ptr(ptr) { }
    ~GOwnPtr() { freeOwnedGPtr(m_ptr); }

    T* get() const { return m_ptr; }
    T* release()
    {
        T* ptr = m_ptr;
        m_ptr = 0;
        return ptr;
    }

    T*& outPtr()
    {
        ASSERT(!m_ptr);
        return m_ptr;
    }

    void set(T* ptr)
    {
        ASSERT(!ptr || m_ptr != ptr);
        freeOwnedGPtr(m_ptr);
        m_ptr = ptr;
    }

    void clear()
    {
        freeOwnedGPtr(m_ptr);
        m_ptr = 0;
    }

    T& operator*() const
    {
        ASSERT(m_ptr);
        return *m_ptr;
    }

    T* operator->() const
    {
        ASSERT(m_ptr);
        return m_ptr;
    }

    bool operator!() const { return !m_ptr; }

    // This conversion operator allows implicit conversion to bool but not to other integer types.
    typedef T* GOwnPtr::*UnspecifiedBoolType;
    operator UnspecifiedBoolType() const { return m_ptr ? &GOwnPtr::m_ptr : 0; }

    void swap(GOwnPtr& o) { std::swap(m_ptr, o.m_ptr); }

private:
    T* m_ptr;
};

template <typename T> inline void swap(GOwnPtr<T>& a, GOwnPtr<T>& b)
{
    a.swap(b);
}

template <typename T, typename U> inline bool operator==(const GOwnPtr<T>& a, U* b)
{ 
    return a.get() == b; 
}

template <typename T, typename U> inline bool operator==(T* a, const GOwnPtr<U>& b) 
{
    return a == b.get(); 
}

template <typename T, typename U> inline bool operator!=(const GOwnPtr<T>& a, U* b)
{
    return a.get() != b; 
}

template <typename T, typename U> inline bool operator!=(T* a, const GOwnPtr<U>& b)
{ 
    return a != b.get(); 
}

template <typename T> inline typename GOwnPtr<T>::PtrType getPtr(const GOwnPtr<T>& p)
{
    return p.get();
}

template <typename T> inline void freeOwnedGPtr(T* ptr)
{
    g_free(ptr);
}

} // namespace WTF

using WTF::GOwnPtr;

#endif // GOwnPtr_h
