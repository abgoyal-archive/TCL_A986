

#include "config.h"
#include "Pattern.h"

#include "AffineTransform.h"
#include "GraphicsContext.h"

namespace WebCore {

QBrush Pattern::createPlatformPattern(const AffineTransform&) const
{
    QPixmap* pixmap = tileImage()->nativeImageForCurrentFrame();
    if (!pixmap)
        return QBrush();

    // Qt merges patter space and user space itself
    QBrush brush(*pixmap);
    brush.setTransform(m_patternSpaceTransformation);

    return brush;
}

}
