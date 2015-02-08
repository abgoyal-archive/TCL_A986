

#include "config.h"
#include "JSStringRefBSTR.h"

#include "JSStringRef.h"

JSStringRef JSStringCreateWithBSTR(BSTR string)
{
    return JSStringCreateWithCharacters(string ? string : L"", string ? SysStringLen(string) : 0);
}

BSTR JSStringCopyBSTR(const JSStringRef string)
{
    return SysAllocStringLen(JSStringGetCharactersPtr(string), JSStringGetLength(string));
}
