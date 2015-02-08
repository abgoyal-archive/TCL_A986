

#include "config.h"
#include "Icon.h"

#include "GraphicsContext.h"
#include "PlatformString.h"
#include "IntRect.h"

#include <qpainter.h>
#include <qpixmap.h>
#include <qrect.h>
#include <qglobal.h>

namespace WebCore {

Icon::Icon()
{
}

Icon::~Icon()
{
}

PassRefPtr<Icon> Icon::createIconForFiles(const Vector<String>& filenames)
{
    if (filenames.isEmpty())
        return 0;

    if (filenames.size() == 1) {
        RefPtr<Icon> i = adoptRef(new Icon);
        i->m_icon = QIcon(filenames[0]);
        return i.release();
    }

    //FIXME: Implement this
    return 0;
}

void Icon::paint(GraphicsContext* ctx, const IntRect& rect)
{
    QPixmap px = m_icon.pixmap(rect.size());
    QPainter *p = static_cast<QPainter*>(ctx->platformContext());
    if (p && !px.isNull())
        p->drawPixmap(rect.x(), rect.y(), px);
}

}

// vim: ts=4 sw=4 et
