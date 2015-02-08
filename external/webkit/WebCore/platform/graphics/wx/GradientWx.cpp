

#include "config.h"
#include "Gradient.h"

#include "CSSParser.h"
#include "NotImplemented.h"

namespace WebCore {

void Gradient::platformDestroy()
{
    notImplemented();
}

PlatformGradient Gradient::platformGradient()
{
    notImplemented();
    return 0;
}

void Gradient::fill(GraphicsContext*, const FloatRect&)
{
    notImplemented();
}

} //namespace
