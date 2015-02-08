

#include "config.h"
#include "StillImageQt.h"

#include "GraphicsContext.h"
#include "IntSize.h"

#include <QPainter>

namespace WebCore {

StillImage::StillImage(const QPixmap& pixmap)
    : m_pixmap(pixmap)
{}

IntSize StillImage::size() const
{
    return IntSize(m_pixmap.width(), m_pixmap.height());
}

NativeImagePtr StillImage::nativeImageForCurrentFrame()
{
    return const_cast<NativeImagePtr>(&m_pixmap);
}

void StillImage::draw(GraphicsContext* ctxt, const FloatRect& dst,
                      const FloatRect& src, ColorSpace, CompositeOperator op)
{
    if (m_pixmap.isNull())
        return;

    ctxt->save();
    ctxt->setCompositeOperation(op);
    QPainter* painter(ctxt->platformContext());
    painter->drawPixmap(dst, m_pixmap, src);
    ctxt->restore();
}

}
