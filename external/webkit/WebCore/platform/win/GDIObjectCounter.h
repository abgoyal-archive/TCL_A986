

#ifndef GDIObjectCounter_h
#define GDIObjectCounter_h

#ifdef NDEBUG
#define LOCAL_GDI_COUNTER(num, identifier) ((void)0)
#else
#define LOCAL_GDI_COUNTER(num, identifier) GDIObjectCounter counter##num(identifier)
#endif

#ifndef NDEBUG

#include "PlatformString.h"

namespace WebCore {

    class GDIObjectCounter {
    public:
        GDIObjectCounter(const String& identifier);
        GDIObjectCounter(const String& className, void* instance);
        ~GDIObjectCounter();

        static unsigned currentGDIObjectsInUse();

    private:
        void init(const String& identifier);
        String m_identifier;
        unsigned m_startCount;
        unsigned m_endCount;
    };

} // namespace WebCore

#endif // !defined(NDEBUG)

#endif // !defined(GDIObjectCounter_h)
