

#include "config.h"

#include "ThreadSpecific.h"
#include <wtf/Noncopyable.h>

#if USE(PTHREADS)
#error This file should not be compiled by ports that do not use Windows native ThreadSpecific implementation.
#endif

namespace WTF {

long& tlsKeyCount()
{
    static long count;
    return count;
}

DWORD* tlsKeys()
{
    static DWORD keys[kMaxTlsKeySize];
    return keys;
}

void ThreadSpecificThreadExit()
{
    for (long i = 0; i < tlsKeyCount(); i++) {
        // The layout of ThreadSpecific<T>::Data does not depend on T. So we are safe to do the static cast to ThreadSpecific<int> in order to access its data member.
        ThreadSpecific<int>::Data* data = static_cast<ThreadSpecific<int>::Data*>(TlsGetValue(tlsKeys()[i]));
        if (data)
            data->destructor(data);
    }
}

} // namespace WTF
