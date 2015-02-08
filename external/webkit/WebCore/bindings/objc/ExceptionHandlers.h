

#ifndef ExceptionHandlers_h
#define ExceptionHandlers_h

#include <wtf/Assertions.h>

#if !defined(NDEBUG) && !defined(DISABLE_THREAD_CHECK)
#define DOM_ASSERT_MAIN_THREAD() do \
    if (!pthread_main_np()) { \
        WTFReportAssertionFailure(__FILE__, __LINE__, WTF_PRETTY_FUNCTION, "DOM access on non-main thread -- you will probably crash soon!"); \
    } \
while (0)
#else
#define DOM_ASSERT_MAIN_THREAD() ((void)0)
#endif

namespace WebCore {
    
    typedef int ExceptionCode;
    
    class SelectionController;
    class Range;

    void raiseDOMException(ExceptionCode);

    inline void raiseOnDOMError(ExceptionCode ec) 
    {
        if (ec) 
            raiseDOMException(ec);
    }

} // namespace WebCore

#endif // ExceptionHandlers_h
