

#ifndef ImageBufferData_h
#define ImageBufferData_h

#include <QPainter>
#include <QPixmap>

#include "OwnPtr.h"

namespace WebCore {

class IntSize;

class ImageBufferData {
public:
    ImageBufferData(const IntSize&);

    QPixmap m_pixmap;
    OwnPtr<QPainter> m_painter;
};

}  // namespace WebCore

#endif  // ImageBufferData_h
