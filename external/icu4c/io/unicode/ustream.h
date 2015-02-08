   
#ifndef USTREAM_H
#define USTREAM_H

#include "unicode/unistr.h"


#if U_IOSTREAM_SOURCE >= 199711
#include <istream>
#include <ostream>

U_NAMESPACE_BEGIN

U_IO_API std::ostream & U_EXPORT2 operator<<(std::ostream& stream, const UnicodeString& s);

U_IO_API std::istream & U_EXPORT2 operator>>(std::istream& stream, UnicodeString& s);
U_NAMESPACE_END

#elif U_IOSTREAM_SOURCE >= 198506
/* <istream.h> and <ostream.h> don't exist. */
#include <iostream.h>

U_NAMESPACE_BEGIN
U_IO_API ostream & U_EXPORT2 operator<<(ostream& stream, const UnicodeString& s);

U_IO_API istream & U_EXPORT2 operator>>(istream& stream, UnicodeString& s);
U_NAMESPACE_END

#endif

/* No operator for UChar because it can conflict with wchar_t  */

#endif
