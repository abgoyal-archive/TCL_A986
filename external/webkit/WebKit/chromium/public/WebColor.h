

#ifndef WebColor_h
#define WebColor_h

#include "WebColorName.h"
#include "WebCommon.h"

namespace WebKit {

typedef unsigned WebColor;  // RGBA quad. Equivalent to SkColor.

// Sets the values of a set of named colors.
WEBKIT_API void setNamedColors(const WebColorName*, const WebColor*, size_t length);

} // namespace WebKit

#endif
