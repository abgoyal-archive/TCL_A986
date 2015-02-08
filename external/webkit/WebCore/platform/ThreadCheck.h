

#ifndef ThreadCheck_h
#define ThreadCheck_h

namespace WebCore {
    enum ThreadViolationBehavior {
        NoThreadCheck,
        LogOnFirstThreadViolation,
        LogOnThreadViolation,
        RaiseExceptionOnThreadViolation
    };
    enum ThreadViolationRound {
        ThreadViolationRoundOne = 0,
        ThreadViolationRoundTwo,
        MaximumThreadViolationRound
    };
    void setDefaultThreadViolationBehavior(ThreadViolationBehavior, ThreadViolationRound);
    void reportThreadViolation(const char* function, ThreadViolationRound);
}

extern "C" void WebCoreReportThreadViolation(const char* function, WebCore::ThreadViolationRound);

#define WebCoreThreadViolationCheckRoundOne() ::WebCore::reportThreadViolation(WTF_PRETTY_FUNCTION, WebCore::ThreadViolationRoundOne)
#define WebCoreThreadViolationCheckRoundTwo() ::WebCore::reportThreadViolation(WTF_PRETTY_FUNCTION, WebCore::ThreadViolationRoundTwo)

#endif
