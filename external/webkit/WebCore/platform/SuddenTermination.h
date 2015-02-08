

#ifndef SuddenTermination_h
#define SuddenTermination_h

#include <wtf/Platform.h>

namespace WebCore {

    // Once disabled via one or more more calls to disableSuddenTermination(), fast shutdown
    // is not valid until enableSuddenTermination() has been called an equal number of times.
    // On Mac, these are thin wrappers around Mac OS X functions of the same name.
    void disableSuddenTermination();
    void enableSuddenTermination();

#if (!PLATFORM(MAC) || defined(BUILDING_ON_TIGER) || defined(BUILDING_ON_LEOPARD)) && !PLATFORM(CHROMIUM)
    inline void disableSuddenTermination() { }
    inline void enableSuddenTermination() { }
#endif

} // namespace WebCore

#endif // SuddenTermination_h
