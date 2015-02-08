

#include "config.h"
#include "BString.h"

#include "AtomicString.h"
#include "KURL.h"
#include "PlatformString.h"
#include <tchar.h>
#include <windows.h>

#if PLATFORM(CF)
#include <CoreFoundation/CoreFoundation.h>
#endif

using namespace JSC;

namespace WebCore {

BString::BString()
    : m_bstr(0)
{
}

BString::BString(const wchar_t* characters)
{
    if (!characters)
        m_bstr = 0;
    else
        m_bstr = SysAllocString(characters);
}

BString::BString(const wchar_t* characters, size_t length)
{
    if (!characters)
        m_bstr = 0;
    else
        m_bstr = SysAllocStringLen(characters, length);
}

BString::BString(const String& s)
{
    if (s.isNull())
        m_bstr = 0;
    else
        m_bstr = SysAllocStringLen(s.characters(), s.length());
}

BString::BString(const KURL& url)
{
    if (url.isNull())
        m_bstr = 0;
    else
        m_bstr = SysAllocStringLen(url.string().characters(), url.string().length());
}

BString::BString(const AtomicString& s)
{
    if (s.isNull())
        m_bstr = 0;
    else
        m_bstr = SysAllocStringLen(s.characters(), s.length());
}

BString::BString(const UString& s)
{
    if (s.isNull())
        m_bstr = 0;
    else
        m_bstr = SysAllocStringLen(s.data(), s.size());
}

#if PLATFORM(CF)
BString::BString(CFStringRef cfstr)
    : m_bstr(0)
{
    if (!cfstr)
        return;

    const UniChar* uniChars = CFStringGetCharactersPtr(cfstr);
    if (uniChars) {
        m_bstr = SysAllocStringLen((LPCTSTR)uniChars, CFStringGetLength(cfstr));
        return;
    }

    CFIndex length = CFStringGetLength(cfstr);
    m_bstr = SysAllocStringLen(0, length);
    CFStringGetCharacters(cfstr, CFRangeMake(0, length), (UniChar*)m_bstr);
    m_bstr[length] = 0;
}
#endif

BString::~BString()
{
    SysFreeString(m_bstr);
}

BString::BString(const BString& other)
{
    if (!other.m_bstr)
        m_bstr = 0;
    else
        m_bstr = SysAllocString(other.m_bstr);
}

void BString::adoptBSTR(BSTR bstr)
{
    if (m_bstr)
        SysFreeString(m_bstr);
    m_bstr = bstr;
}

BString& BString::operator=(const BString& other)
{
    if (this != &other)
        *this = other.m_bstr;
    return *this;
}

BString& BString::operator=(const BSTR& other)
{
    if (other != m_bstr) {
        SysFreeString(m_bstr);
        m_bstr = other ? SysAllocString(other) : 0;
    }

    return *this;
}

bool operator ==(const BString& a, const BString& b)
{
    if (SysStringLen((BSTR)a) != SysStringLen((BSTR)b))
        return false;
    if (!(BSTR)a && !(BSTR)b)
        return true;
    if (!(BSTR)a || !(BSTR)b)
        return false;
    return !_tcscmp((BSTR)a, (BSTR)b);
}

bool operator !=(const BString& a, const BString& b)
{
    return !(a==b);
}

bool operator ==(const BString& a, BSTR b)
{
    if (SysStringLen((BSTR)a) != SysStringLen(b))
        return false;
    if (!(BSTR)a && !b)
        return true;
    if (!(BSTR)a || !b)
        return false;
    return !_tcscmp((BSTR)a, b);
}

bool operator !=(const BString& a, BSTR b)
{
    return !(a==b);
}

bool operator ==(BSTR a, const BString& b)
{
    if (SysStringLen(a) != SysStringLen((BSTR)b))
        return false;
    if (!a && !(BSTR)b)
        return true;
    if (!a || !(BSTR)b)
        return false;
    return !_tcscmp(a, (BSTR)b);
}

bool operator !=(BSTR a, const BString& b)
{
    return !(a==b);
}

}
