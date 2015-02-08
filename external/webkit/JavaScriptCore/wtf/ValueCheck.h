

#ifndef ValueCheck_h
#define ValueCheck_h

// For malloc_size and _msize.
#if OS(DARWIN)
#include <malloc/malloc.h>
#elif COMPILER(MSVC)
#include <malloc.h>
#endif

namespace WTF {

template<typename T> struct ValueCheck {
    typedef T TraitType;
    static void checkConsistency(const T&) { }
};

#if !ASSERT_DISABLED
template<typename P> struct ValueCheck<P*> {
    typedef P* TraitType;
    static void checkConsistency(const P* p)
    {
        if (!p)
            return;
#if (defined(USE_SYSTEM_MALLOC) && USE_SYSTEM_MALLOC) || !defined(NDEBUG)
#if OS(DARWIN)
        ASSERT(malloc_size(p));
#elif COMPILER(MSVC)
        ASSERT(_msize(const_cast<P*>(p)));
#endif
#endif
        ValueCheck<P>::checkConsistency(*p);
    }
};
#endif

}

#endif // ValueCheck_h
