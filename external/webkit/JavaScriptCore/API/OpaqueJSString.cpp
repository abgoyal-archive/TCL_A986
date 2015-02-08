

#include "config.h"
#include "OpaqueJSString.h"

#include <interpreter/CallFrame.h>
#include <runtime/JSGlobalObject.h>
#include <runtime/Identifier.h>

using namespace JSC;

PassRefPtr<OpaqueJSString> OpaqueJSString::create(const UString& ustring)
{
    if (!ustring.isNull())
        return adoptRef(new OpaqueJSString(ustring.data(), ustring.size()));
    return 0;
}

UString OpaqueJSString::ustring() const
{
    if (this && m_characters)
        return UString(m_characters, m_length);
    return UString::null();
}

Identifier OpaqueJSString::identifier(JSGlobalData* globalData) const
{
    if (!this || !m_characters)
        return Identifier(globalData, static_cast<const char*>(0));

    return Identifier(globalData, m_characters, m_length);
}
