

#include "config.h"

#ifndef NDEBUG

#include "GDIObjectCounter.h"

#include "CString.h"
#include "Logging.h"

#include <windows.h>

namespace WebCore {

GDIObjectCounter::GDIObjectCounter(const String& identifier)
{
    init(identifier);
}

GDIObjectCounter::GDIObjectCounter(const String& className, void* instance)
{
    init(String::format("%s (%p)", className.latin1().data(), instance));
}

void GDIObjectCounter::init(const String& identifier)
{
    m_identifier = identifier;
    m_startCount = currentGDIObjectsInUse();
    m_endCount = 0;
}

GDIObjectCounter::~GDIObjectCounter()
{
    m_endCount = currentGDIObjectsInUse();
    int leaked = m_endCount - m_startCount;
    if (leaked != 0)
        LOG(PlatformLeaks, "%s: leaked %d GDI object%s!", m_identifier.latin1().data(), leaked, leaked == 1 ? "" : "s");
}

unsigned GDIObjectCounter::currentGDIObjectsInUse()
{
    return ::GetGuiResources(::GetCurrentProcess(), GR_GDIOBJECTS);
}

} // namespace WebCore

#endif // !defined(NDEBUG)
