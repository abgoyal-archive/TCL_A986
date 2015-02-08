

#ifndef OwnPtrBrew_h
#define OwnPtrBrew_h

#include <algorithm>
#include <wtf/Assertions.h>
#include <wtf/Noncopyable.h>

// Forward delcarations at this point avoid the need to include BREW includes
// in WTF headers.
typedef struct _IFileMgr IFileMgr;
typedef struct _IFile IFile;
typedef struct IBitmap IBitmap;

namespace WTF {

template <typename T> void freeOwnedPtrBrew(T* ptr);
template<> void freeOwnedPtrBrew<IFileMgr>(IFileMgr*);
template<> void freeOwnedPtrBrew<IFile>(IFile*);
template<> void freeOwnedPtrBrew<IBitmap>(IBitmap*);

template <typename T> class OwnPtrBrew : public Noncopyable {
public:
    explicit OwnPtrBrew(T* ptr = 0) : m_ptr(ptr) { }
    ~OwnPtrBrew() { freeOwnedPtrBrew(m_ptr); }

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
        freeOwnedPtrBrew(m_ptr);
        m_ptr = ptr;
    }

    void clear()
    {
        freeOwnedPtrBrew(m_ptr);
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
    typedef T* OwnPtrBrew::*UnspecifiedBoolType;
    operator UnspecifiedBoolType() const { return m_ptr ? &OwnPtrBrew::m_ptr : 0; }

    void swap(OwnPtrBrew& o) { std::swap(m_ptr, o.m_ptr); }

private:
    T* m_ptr;
};

template <typename T> inline void swap(OwnPtrBrew<T>& a, OwnPtrBrew<T>& b)
{
    a.swap(b);
}

template <typename T, typename U> inline bool operator==(const OwnPtrBrew<T>& a, U* b)
{
    return a.get() == b;
}

template <typename T, typename U> inline bool operator==(T* a, const OwnPtrBrew<U>& b)
{
    return a == b.get();
}

template <typename T, typename U> inline bool operator!=(const OwnPtrBrew<T>& a, U* b)
{
    return a.get() != b;
}

template <typename T, typename U> inline bool operator!=(T* a, const OwnPtrBrew<U>& b)
{
    return a != b.get();
}

template <typename T> inline typename OwnPtrBrew<T>::PtrType getPtr(const OwnPtrBrew<T>& p)
{
    return p.get();
}

} // namespace WTF

using WTF::OwnPtrBrew;

#endif // OwnPtrBrew_h
