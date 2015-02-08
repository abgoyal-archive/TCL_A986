

#ifndef WTF_RandomNumberSeed_h
#define WTF_RandomNumberSeed_h

#include <stdlib.h>
#include <time.h>

#if HAVE(SYS_TIME_H)
#include <sys/time.h>
#endif

#if OS(UNIX)
#include <sys/types.h>
#include <unistd.h>
#endif

#if OS(WINCE)
extern "C" {
void init_by_array(unsigned long init_key[],int key_length);
}
#endif

// Internal JavaScriptCore usage only
namespace WTF {

inline void initializeRandomNumberGenerator()
{
#if OS(DARWIN)
    // On Darwin we use arc4random which initialises itself.
#elif OS(WINCE)
    // initialize rand()
    srand(static_cast<unsigned>(time(0)));

    // use rand() to initialize the real RNG
    unsigned long initializationBuffer[4];
    initializationBuffer[0] = (rand() << 16) | rand();
    initializationBuffer[1] = (rand() << 16) | rand();
    initializationBuffer[2] = (rand() << 16) | rand();
    initializationBuffer[3] = (rand() << 16) | rand();
    init_by_array(initializationBuffer, 4);
#elif COMPILER(MSVC) && defined(_CRT_RAND_S)
    // On Windows we use rand_s which initialises itself
#elif OS(UNIX)
    // srandomdev is not guaranteed to exist on linux so we use this poor seed, this should be improved
    timeval time;
    gettimeofday(&time, 0);
    srandom(static_cast<unsigned>(time.tv_usec * getpid()));
#else
    srand(static_cast<unsigned>(time(0)));
#endif
}

inline void initializeWeakRandomNumberGenerator()
{
#if COMPILER(MSVC) && defined(_CRT_RAND_S)
    // We need to initialise windows rand() explicitly for Math.random
    unsigned seed = 0;
    rand_s(&seed);
    srand(seed);
#endif
}
}

#endif
