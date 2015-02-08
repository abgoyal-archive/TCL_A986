

#ifndef WebCanvas_h
#define WebCanvas_h

#include "WebCommon.h"

#if WEBKIT_USING_SKIA
namespace skia { class PlatformCanvas; }
#elif WEBKIT_USING_CG
struct CGContext;
#endif

namespace WebKit {

#if WEBKIT_USING_SKIA
typedef skia::PlatformCanvas WebCanvas;
#elif WEBKIT_USING_CG
typedef struct CGContext WebCanvas;
#else
#error "Need to define WebCanvas"
#endif

} // namespace WebKit

#endif
