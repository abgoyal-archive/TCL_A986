
#include "unicode/utypes.h"

#if !UCONFIG_NO_FORMATTING && !UCONFIG_NO_CONVERSION

#include "unicode/fmtable.h"

// *****************************************************************************
// class Formattable
// *****************************************************************************

U_NAMESPACE_BEGIN

// -------------------------------------
// Creates a formattable object with a char* string.
// This API is useless. The API that takes a UnicodeString is actually just as good.
// This is just a grandfathered API.

Formattable::Formattable(const char* stringToCopy)
{
    init();
    fType = kString;
    fValue.fString = new UnicodeString(stringToCopy);
}

U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_FORMATTING || !UCONFIG_NO_CONVERSION */

//eof
