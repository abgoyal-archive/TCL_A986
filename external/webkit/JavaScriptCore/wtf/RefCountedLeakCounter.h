
 
#ifndef RefCountedLeakCounter_h
#define RefCountedLeakCounter_h
 
#include "Assertions.h"
#include "Threading.h"

namespace WTF {
    
    struct RefCountedLeakCounter {
        static void suppressMessages(const char*);
        static void cancelMessageSuppression(const char*);
        
        explicit RefCountedLeakCounter(const char* description);
        ~RefCountedLeakCounter();

        void increment();
        void decrement();

#ifndef NDEBUG
    private:
        volatile int m_count;
        const char* m_description;
#endif
    };

}  // namespace WTF

#endif
