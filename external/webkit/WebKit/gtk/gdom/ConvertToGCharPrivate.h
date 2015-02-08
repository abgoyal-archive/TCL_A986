

#ifndef ConvertToGCharPrivate_h
#define ConvertToGCharPrivate_h

#include "AtomicString.h"
#include "CString.h"
#include "KURL.h"
#include "PlatformString.h"

inline gchar* copyAsGchar(WebCore::String const& s)
{
    return g_strdup(s.utf8().data());
}

inline gchar* copyAsGchar(WebCore::KURL const& s)
{
    return copyAsGchar(s.string());
}

inline gchar* copyAsGchar(const JSC::UString& s)
{
    return g_strdup(s.UTF8String().c_str());
}

inline gchar* copyAsGchar(WebCore::AtomicString const& s)
{
    return g_strdup(s.string().utf8().data());
}

#endif /* ConvertToGCharPrivate_h*/
