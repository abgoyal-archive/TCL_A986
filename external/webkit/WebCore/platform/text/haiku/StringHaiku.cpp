

#include "config.h"
#include "PlatformString.h"

#include "CString.h"
#include <String.h>

namespace WebCore {

// String conversions
String::String(const BString& bstring)
{
    const UChar* str = reinterpret_cast<const UChar*>(bstring.String());
    const size_t size = bstring.Length();

    if (!str)
        return;

    if (!size)
        m_impl = StringImpl::empty();
    else
        m_impl = StringImpl::create(str, size);
}

String::operator BString() const
{
    BString string;
    string.SetTo(utf8().data());

    return string;
}

} // namespace WebCore

