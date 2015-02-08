

#include "config.h"
#include "Icon.h"

#include <windows.h>
#include <shellapi.h>

#include "GraphicsContext.h"
#include "PlatformContextSkia.h"
#include "PlatformString.h"
#include "SkiaUtils.h"

namespace WebCore {

Icon::Icon(const PlatformIcon& icon)
    : m_icon(icon)
{
}

Icon::~Icon()
{
    if (m_icon)
        DestroyIcon(m_icon);
}

PassRefPtr<Icon> Icon::createIconForFiles(const Vector<String>& filenames)
{
    // FIXME: We can't access icons directly from renderer processes.
    // http://code.google.com/p/chromium/issues/detail?id=4092
    return 0;
}

void Icon::paint(GraphicsContext* context, const IntRect& rect)
{
    if (context->paintingDisabled())
        return;

    HDC hdc = context->platformContext()->canvas()->beginPlatformPaint();
    DrawIconEx(hdc, rect.x(), rect.y(), m_icon, rect.width(), rect.height(), 0, 0, DI_NORMAL);
    context->platformContext()->canvas()->endPlatformPaint();
}

} // namespace WebCore
