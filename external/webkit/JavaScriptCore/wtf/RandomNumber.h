

#ifndef WTF_RandomNumber_h
#define WTF_RandomNumber_h

namespace WTF {

    // Returns a pseudo-random number in the range [0, 1), attempts to be
    // cryptographically secure if possible on the target platform
    double randomNumber();

    // Returns a pseudo-random number in the range [0, 1), attempts to
    // produce a reasonable "random" number fast.
    // We only need this because rand_s is so slow on windows.
    double weakRandomNumber();

}

#endif
