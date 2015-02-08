

#ifndef WTF_Forward_h
#define WTF_Forward_h

#include <stddef.h>

namespace WTF {
    template<typename T> class ListRefPtr;
    template<typename T> class OwnArrayPtr;
    template<typename T> class OwnPtr;
    template<typename T> class PassOwnPtr;
    template<typename T> class PassRefPtr;
    template<typename T> class RefPtr;
    template<typename T, size_t inlineCapacity> class Vector;
}

using WTF::ListRefPtr;
using WTF::OwnArrayPtr;
using WTF::OwnPtr;
using WTF::PassOwnPtr;
using WTF::PassRefPtr;
using WTF::RefPtr;
using WTF::Vector;

#endif // WTF_Forward_h
