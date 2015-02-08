

#include "config.h"
#include "Icon.h"

#include "GraphicsContext.h"
#include "NotImplemented.h"
#include "PlatformString.h"

namespace WebCore {

Icon::Icon(const PlatformIcon& icon)
    : m_icon(icon)
{
}

Icon::~Icon()
{
}

PassRefPtr<Icon> Icon::createIconForFiles(const Vector<String>&)
{
    notImplemented();
    return 0;
}

void Icon::paint(GraphicsContext*, const IntRect&)
{
    notImplemented();
}

} // namespace WebCore
