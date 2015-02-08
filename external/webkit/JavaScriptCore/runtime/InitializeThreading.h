
 
#ifndef InitializeThreading_h
#define InitializeThreading_h

namespace JSC {

    // This function must be called from the main thread. It is safe to call it repeatedly.
    // Darwin is an exception to this rule: it is OK to call this function from any thread, even reentrantly.
    void initializeThreading();

}

#endif // InitializeThreading_h
