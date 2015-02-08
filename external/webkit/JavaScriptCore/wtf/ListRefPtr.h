

#ifndef WTF_ListRefPtr_h
#define WTF_ListRefPtr_h

#include <wtf/RefPtr.h>

namespace WTF {

    // Specialized version of RefPtr desgined for use in singly-linked lists. 
    // Derefs the list iteratively to avoid recursive derefing that can overflow the stack.
    template <typename T> class ListRefPtr : public RefPtr<T> {
    public:
        ListRefPtr() : RefPtr<T>() {}
        ListRefPtr(T* ptr) : RefPtr<T>(ptr) {}
        ListRefPtr(const RefPtr<T>& o) : RefPtr<T>(o) {}
        // see comment in PassRefPtr.h for why this takes const reference
        template <typename U> ListRefPtr(const PassRefPtr<U>& o) : RefPtr<T>(o) {}
        
        ~ListRefPtr();
        
        ListRefPtr& operator=(T* optr) { RefPtr<T>::operator=(optr); return *this; }
        ListRefPtr& operator=(const RefPtr<T>& o) { RefPtr<T>::operator=(o); return *this; }
        ListRefPtr& operator=(const PassRefPtr<T>& o) { RefPtr<T>::operator=(o); return *this; }
        template <typename U> ListRefPtr& operator=(const RefPtr<U>& o) { RefPtr<T>::operator=(o); return *this; }
        template <typename U> ListRefPtr& operator=(const PassRefPtr<U>& o) { RefPtr<T>::operator=(o); return *this; }
    };

    // Remove inline for winscw compiler to prevent the compiler agressively resolving
    // T::ref() in RefPtr<T>'s copy constructor.  The bug is reported at:
    // https://xdabug001.ext.nokia.com/bugzilla/show_bug.cgi?id=9812.
    template <typename T> 
#if !COMPILER(WINSCW)
    inline
#endif
    ListRefPtr<T>::~ListRefPtr()
    {
        RefPtr<T> reaper = this->release();
        while (reaper && reaper->hasOneRef())
            reaper = reaper->releaseNext(); // implicitly protects reaper->next, then derefs reaper
    }

    template <typename T> inline T* getPtr(const ListRefPtr<T>& p)
    {
        return p.get();
    }

} // namespace WTF

using WTF::ListRefPtr;

#endif // WTF_ListRefPtr_h
