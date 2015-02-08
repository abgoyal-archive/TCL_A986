

#ifndef NotImplemented_h
#define NotImplemented_h

#include "Logging.h"
#include <wtf/Assertions.h>

#if PLATFORM(GTK)
    #define supressNotImplementedWarning() getenv("DISABLE_NI_WARNING")
#elif PLATFORM(QT)
    #include <QByteArray>
    #define supressNotImplementedWarning() !qgetenv("DISABLE_NI_WARNING").isEmpty()
#else
    #define supressNotImplementedWarning() false
#endif

#if defined ANDROID

    #if 1 && defined LOG_TAG
        #ifndef _LIBS_UTILS_LOG_H
            #undef LOG
            #include <utils/Log.h>
        #endif
        #define notImplemented() LOGV("%s: notImplemented\n", __PRETTY_FUNCTION__)
        #define lowPriority_notImplemented() //printf("%s\n", __PRETTY_FUNCTION__)
        #define verifiedOk()    // not a problem that it's not implemented
    #else
        #define notImplemented() fprintf(stderr, "%s\n", __PRETTY_FUNCTION__)
    #endif
#elif defined(NDEBUG)
    #define notImplemented() ((void)0)
#else

#define notImplemented() do { \
        static bool havePrinted = false; \
        if (!havePrinted && !supressNotImplementedWarning()) { \
            WTFLogVerbose(__FILE__, __LINE__, WTF_PRETTY_FUNCTION, &::WebCore::LogNotYetImplemented, "UNIMPLEMENTED: "); \
            havePrinted = true; \
        } \
    } while (0)

#endif // NDEBUG

#endif // NotImplemented_h
