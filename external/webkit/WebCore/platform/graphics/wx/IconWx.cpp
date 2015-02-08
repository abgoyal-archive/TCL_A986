

#include "config.h"
#include "Icon.h"

#include "GraphicsContext.h"
#include "PlatformString.h"
#include "IntRect.h"
#include "NotImplemented.h"

namespace WebCore {

Icon::~Icon()
{
}

PassRefPtr<Icon> Icon::createIconForFiles(const Vector<String>& filenames)
{
    notImplemented();
    return 0;
}

void Icon::paint(GraphicsContext* ctx, const IntRect& rect)
{
    notImplemented();
}

}

