

#ifndef CurrentTime_h
#define CurrentTime_h

#include <time.h>

namespace WTF {

    // Returns the current UTC time in seconds, counted from January 1, 1970.
    // Precision varies depending on platform but is usually as good or better 
    // than a millisecond.
    double currentTime();

    // Same thing, in milliseconds.
    inline double currentTimeMS()
    {
        return currentTime() * 1000.0; 
    }

    inline void getLocalTime(const time_t* localTime, struct tm* localTM)
    {
    #if COMPILER(MSVC7) || COMPILER(MINGW) || OS(WINCE)
        *localTM = *localtime(localTime);
    #elif COMPILER(MSVC)
        localtime_s(localTM, localTime);
    #else
        localtime_r(localTime, localTM);
    #endif
    }

} // namespace WTF

using WTF::currentTime;
using WTF::getLocalTime;

#endif // CurrentTime_h

