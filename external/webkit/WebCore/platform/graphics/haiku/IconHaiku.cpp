

#include "config.h"
#include "Icon.h"

#include "GraphicsContext.h"
#include "IntRect.h"
#include "NotImplemented.h"
#include "PlatformString.h"


namespace WebCore {

Icon::~Icon()
{
    notImplemented();
}

PassRefPtr<Icon> Icon::createIconForFiles(const Vector<String>& filenames)
{
    notImplemented();
    return 0;
}

void Icon::paint(GraphicsContext*, const IntRect&)
{
    notImplemented();
}

} // namespace WebCore

