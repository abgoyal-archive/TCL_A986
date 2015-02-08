

#ifndef RetainPtr_h
#define RetainPtr_h

#include "TypeTraits.h"
#include <algorithm>
#include <CoreFoundation/CoreFoundation.h>

#ifdef __OBJC__
#import <Foundation/Foundation.h>
#endif

namespace WTF {

    // Unlike most most of our smart pointers, RetainPtr can take either the pointer type or the pointed-to type,
    // so both RetainPtr<NSDictionary> and RetainPtr<CFDictionaryRef> will work.

    enum AdoptCFTag { AdoptCF };
    enum AdoptNSTag { AdoptNS };
    
#ifdef __OBJC__
    inline void adoptNSReference(id ptr)
    {
        if (ptr) {
            CFRetain(ptr);
            [ptr release];
        }
    }
#endif

    template <typename T> class RetainPtr {
    public:
        typedef typename RemovePointer<T>::Type ValueType;
        typedef ValueType* PtrType;

        RetainPtr() : m_ptr(0) {}
        RetainPtr(PtrType ptr) : m_ptr(ptr) { if (ptr) CFRetain(ptr); }

        RetainPtr(AdoptCFTag, PtrType ptr) : m_ptr(ptr) { }
        RetainPtr(AdoptNSTag, PtrType ptr) : m_ptr(ptr) { adoptNSReference(ptr); }
        
        RetainPtr(const RetainPtr& o) : m_ptr(o.m_ptr) { if (PtrType ptr = m_ptr) CFRetain(ptr); }

        ~RetainPtr() { if (PtrType ptr = m_ptr) CFRelease(ptr); }
        
        template <typename U> RetainPtr(const RetainPtr<U>& o) : m_ptr(o.get()) { if (PtrType ptr = m_ptr) CFRetain(ptr); }
        
        PtrType get() const { return m_ptr; }
        
        PtrType releaseRef() { PtrType tmp = m_ptr; m_ptr = 0; return tmp; }
        
        PtrType operator->() const { return m_ptr; }
        
        bool operator!() const { return !m_ptr; }
    
        // This conversion operator allows implicit conversion to bool but not to other integer types.
        typedef PtrType RetainPtr::*UnspecifiedBoolType;
        operator UnspecifiedBoolType() const { return m_ptr ? &RetainPtr::m_ptr : 0; }
        
        RetainPtr& operator=(const RetainPtr&);
        template <typename U> RetainPtr& operator=(const RetainPtr<U>&);
        RetainPtr& operator=(PtrType);
        template <typename U> RetainPtr& operator=(U*);

        void adoptCF(PtrType);
        void adoptNS(PtrType);
        
        void swap(RetainPtr&);

    private:
        PtrType m_ptr;
    };
    
    template <typename T> inline RetainPtr<T>& RetainPtr<T>::operator=(const RetainPtr<T>& o)
    {
        PtrType optr = o.get();
        if (optr)
            CFRetain(optr);
        PtrType ptr = m_ptr;
        m_ptr = optr;
        if (ptr)
            CFRelease(ptr);
        return *this;
    }
    
    template <typename T> template <typename U> inline RetainPtr<T>& RetainPtr<T>::operator=(const RetainPtr<U>& o)
    {
        PtrType optr = o.get();
        if (optr)
            CFRetain(optr);
        PtrType ptr = m_ptr;
        m_ptr = optr;
        if (ptr)
            CFRelease(ptr);
        return *this;
    }
    
    template <typename T> inline RetainPtr<T>& RetainPtr<T>::operator=(PtrType optr)
    {
        if (optr)
            CFRetain(optr);
        PtrType ptr = m_ptr;
        m_ptr = optr;
        if (ptr)
            CFRelease(ptr);
        return *this;
    }

    template <typename T> inline void RetainPtr<T>::adoptCF(PtrType optr)
    {
        PtrType ptr = m_ptr;
        m_ptr = optr;
        if (ptr)
            CFRelease(ptr);
    }

    template <typename T> inline void RetainPtr<T>::adoptNS(PtrType optr)
    {
        adoptNSReference(optr);
        
        PtrType ptr = m_ptr;
        m_ptr = optr;
        if (ptr)
            CFRelease(ptr);
    }
    
    template <typename T> template <typename U> inline RetainPtr<T>& RetainPtr<T>::operator=(U* optr)
    {
        if (optr)
            CFRetain(optr);
        PtrType ptr = m_ptr;
        m_ptr = optr;
        if (ptr)
            CFRelease(ptr);
        return *this;
    }

    template <class T> inline void RetainPtr<T>::swap(RetainPtr<T>& o)
    {
        std::swap(m_ptr, o.m_ptr);
    }

    template <class T> inline void swap(RetainPtr<T>& a, RetainPtr<T>& b)
    {
        a.swap(b);
    }

    template <typename T, typename U> inline bool operator==(const RetainPtr<T>& a, const RetainPtr<U>& b)
    { 
        return a.get() == b.get(); 
    }

    template <typename T, typename U> inline bool operator==(const RetainPtr<T>& a, U* b)
    { 
        return a.get() == b; 
    }
    
    template <typename T, typename U> inline bool operator==(T* a, const RetainPtr<U>& b) 
    {
        return a == b.get(); 
    }
    
    template <typename T, typename U> inline bool operator!=(const RetainPtr<T>& a, const RetainPtr<U>& b)
    { 
        return a.get() != b.get(); 
    }

    template <typename T, typename U> inline bool operator!=(const RetainPtr<T>& a, U* b)
    {
        return a.get() != b; 
    }

    template <typename T, typename U> inline bool operator!=(T* a, const RetainPtr<U>& b)
    { 
        return a != b.get(); 
    }

} // namespace WTF

using WTF::AdoptCF;
using WTF::AdoptNS;
using WTF::RetainPtr;

#endif // WTF_RetainPtr_h
