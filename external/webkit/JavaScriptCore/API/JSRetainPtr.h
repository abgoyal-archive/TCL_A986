

#ifndef JSRetainPtr_h
#define JSRetainPtr_h

#include <JavaScriptCore/JSStringRef.h>
#include <algorithm>

inline void JSRetain(JSStringRef string) { JSStringRetain(string); }
inline void JSRelease(JSStringRef string) { JSStringRelease(string); }

enum AdoptTag { Adopt };

template <typename T> class JSRetainPtr {
public:
    JSRetainPtr() : m_ptr(0) {}
    JSRetainPtr(T ptr) : m_ptr(ptr) { if (ptr) JSRetain(ptr); }

    JSRetainPtr(AdoptTag, T ptr) : m_ptr(ptr) { }
    
    JSRetainPtr(const JSRetainPtr& o) : m_ptr(o.m_ptr) { if (T ptr = m_ptr) JSRetain(ptr); }

    ~JSRetainPtr() { if (T ptr = m_ptr) JSRelease(ptr); }
    
    template <typename U> JSRetainPtr(const JSRetainPtr<U>& o) : m_ptr(o.get()) { if (T ptr = m_ptr) JSRetain(ptr); }
    
    T get() const { return m_ptr; }
    
    T releaseRef() { T tmp = m_ptr; m_ptr = 0; return tmp; }
    
    T operator->() const { return m_ptr; }
    
    bool operator!() const { return !m_ptr; }

    // This conversion operator allows implicit conversion to bool but not to other integer types.
    typedef T JSRetainPtr::*UnspecifiedBoolType;
    operator UnspecifiedBoolType() const { return m_ptr ? &JSRetainPtr::m_ptr : 0; }
    
    JSRetainPtr& operator=(const JSRetainPtr&);
    template <typename U> JSRetainPtr& operator=(const JSRetainPtr<U>&);
    JSRetainPtr& operator=(T);
    template <typename U> JSRetainPtr& operator=(U*);

    void adopt(T);
    
    void swap(JSRetainPtr&);

private:
    T m_ptr;
};

template <typename T> inline JSRetainPtr<T>& JSRetainPtr<T>::operator=(const JSRetainPtr<T>& o)
{
    T optr = o.get();
    if (optr)
        JSRetain(optr);
    T ptr = m_ptr;
    m_ptr = optr;
    if (ptr)
        JSRelease(ptr);
    return *this;
}

template <typename T> template <typename U> inline JSRetainPtr<T>& JSRetainPtr<T>::operator=(const JSRetainPtr<U>& o)
{
    T optr = o.get();
    if (optr)
        JSRetain(optr);
    T ptr = m_ptr;
    m_ptr = optr;
    if (ptr)
        JSRelease(ptr);
    return *this;
}

template <typename T> inline JSRetainPtr<T>& JSRetainPtr<T>::operator=(T optr)
{
    if (optr)
        JSRetain(optr);
    T ptr = m_ptr;
    m_ptr = optr;
    if (ptr)
        JSRelease(ptr);
    return *this;
}

template <typename T> inline void JSRetainPtr<T>::adopt(T optr)
{
    T ptr = m_ptr;
    m_ptr = optr;
    if (ptr)
        JSRelease(ptr);
}

template <typename T> template <typename U> inline JSRetainPtr<T>& JSRetainPtr<T>::operator=(U* optr)
{
    if (optr)
        JSRetain(optr);
    T ptr = m_ptr;
    m_ptr = optr;
    if (ptr)
        JSRelease(ptr);
    return *this;
}

template <class T> inline void JSRetainPtr<T>::swap(JSRetainPtr<T>& o)
{
    std::swap(m_ptr, o.m_ptr);
}

template <class T> inline void swap(JSRetainPtr<T>& a, JSRetainPtr<T>& b)
{
    a.swap(b);
}

template <typename T, typename U> inline bool operator==(const JSRetainPtr<T>& a, const JSRetainPtr<U>& b)
{ 
    return a.get() == b.get(); 
}

template <typename T, typename U> inline bool operator==(const JSRetainPtr<T>& a, U* b)
{ 
    return a.get() == b; 
}

template <typename T, typename U> inline bool operator==(T* a, const JSRetainPtr<U>& b) 
{
    return a == b.get(); 
}

template <typename T, typename U> inline bool operator!=(const JSRetainPtr<T>& a, const JSRetainPtr<U>& b)
{ 
    return a.get() != b.get(); 
}

template <typename T, typename U> inline bool operator!=(const JSRetainPtr<T>& a, U* b)
{
    return a.get() != b; 
}

template <typename T, typename U> inline bool operator!=(T* a, const JSRetainPtr<U>& b)
{ 
    return a != b.get(); 
}


#endif // JSRetainPtr_h
