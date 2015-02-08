

#ifndef CachedResourceHandle_h
#define CachedResourceHandle_h

#include "CachedResource.h"

namespace WebCore {

    class CachedResourceHandleBase {
    public:
        ~CachedResourceHandleBase() { if (m_resource) m_resource->unregisterHandle(this); }
        CachedResource* get() const { return m_resource; }
        
        bool operator!() const { return !m_resource; }
        
        // This conversion operator allows implicit conversion to bool but not to other integer types.
        // Parenthesis is needed for winscw compiler to resolve class qualifier in this case.
        typedef CachedResource* (CachedResourceHandleBase::*UnspecifiedBoolType);
        operator UnspecifiedBoolType() const { return m_resource ? &CachedResourceHandleBase::m_resource : 0; }

    protected:
        CachedResourceHandleBase() : m_resource(0) {}
        CachedResourceHandleBase(CachedResource* res) { m_resource = res; if (m_resource) m_resource->registerHandle(this); }
        CachedResourceHandleBase(const CachedResourceHandleBase& o) : m_resource(o.m_resource) { if (m_resource) m_resource->registerHandle(this); }

        void setResource(CachedResource*);
        
    private:
        CachedResourceHandleBase& operator=(const CachedResourceHandleBase&) { return *this; } 
        
        friend class CachedResource;

        CachedResource* m_resource;
    };
        
    template <class R> class CachedResourceHandle : public CachedResourceHandleBase {
    public: 
        CachedResourceHandle() { }
        CachedResourceHandle(R* res);
        CachedResourceHandle(const CachedResourceHandle<R>& o) : CachedResourceHandleBase(o) { }

        R* get() const { return reinterpret_cast<R*>(CachedResourceHandleBase::get()); }
        R* operator->() const { return get(); }
               
        CachedResourceHandle& operator=(R* res) { setResource(res); return *this; } 
        CachedResourceHandle& operator=(const CachedResourceHandle& o) { setResource(o.get()); return *this; }
        bool operator==(const CachedResourceHandleBase& o) const { return get() == o.get(); }
        bool operator!=(const CachedResourceHandleBase& o) const { return get() != o.get(); }
    };

    // Don't inline for winscw compiler to prevent the compiler aggressively resolving
    // the base class of R* when CachedResourceHandler<T>(R*) is inlined.  The bug is
    // reported at: https://xdabug001.ext.nokia.com/bugzilla/show_bug.cgi?id=9812.
    template <class R>
#if !COMPILER(WINSCW)
    inline
#endif
    CachedResourceHandle<R>::CachedResourceHandle(R* res) : CachedResourceHandleBase(res) 
    {
    }
    
    template <class R, class RR> bool operator==(const CachedResourceHandle<R>& h, const RR* res) 
    { 
        return h.get() == res; 
    }
    template <class R, class RR> bool operator==(const RR* res, const CachedResourceHandle<R>& h) 
    { 
        return h.get() == res; 
    }
    template <class R, class RR> bool operator!=(const CachedResourceHandle<R>& h, const RR* res) 
    { 
        return h.get() != res; 
    }
    template <class R, class RR> bool operator!=(const RR* res, const CachedResourceHandle<R>& h) 
    { 
        return h.get() != res; 
    }
}

#endif
