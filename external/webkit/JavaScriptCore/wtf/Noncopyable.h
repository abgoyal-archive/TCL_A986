

#ifndef WTF_Noncopyable_h
#define WTF_Noncopyable_h

// We don't want argument-dependent lookup to pull in everything from the WTF
// namespace when you use Noncopyable, so put it in its own namespace.

#include "FastAllocBase.h"

namespace WTFNoncopyable {

    class Noncopyable : public FastAllocBase {
        Noncopyable(const Noncopyable&);
        Noncopyable& operator=(const Noncopyable&);
    protected:
        Noncopyable() { }
        ~Noncopyable() { }
    };

    class NoncopyableCustomAllocated {
        NoncopyableCustomAllocated(const NoncopyableCustomAllocated&);
        NoncopyableCustomAllocated& operator=(const NoncopyableCustomAllocated&);
    protected:
        NoncopyableCustomAllocated() { }
        ~NoncopyableCustomAllocated() { }
    };

} // namespace WTFNoncopyable

using WTFNoncopyable::Noncopyable;
using WTFNoncopyable::NoncopyableCustomAllocated;

#endif // WTF_Noncopyable_h
