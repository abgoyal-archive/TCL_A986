

#ifndef ScrollbarThemeQt_h
#define ScrollbarThemeQt_h

#include "ScrollbarTheme.h"

#include <QtCore/qglobal.h>

QT_BEGIN_NAMESPACE
class QStyle;
QT_END_NAMESPACE

namespace WebCore {

class ScrollbarThemeQt : public ScrollbarTheme {
public:
    virtual ~ScrollbarThemeQt();

    virtual bool paint(Scrollbar*, GraphicsContext*, const IntRect& damageRect);
    virtual void paintScrollCorner(ScrollView*, GraphicsContext*, const IntRect& cornerRect);

    virtual ScrollbarPart hitTest(Scrollbar*, const PlatformMouseEvent&);

    virtual bool shouldCenterOnThumb(Scrollbar*, const PlatformMouseEvent&);

    virtual void invalidatePart(Scrollbar*, ScrollbarPart);

    virtual int thumbPosition(Scrollbar*);
    virtual int thumbLength(Scrollbar*);
    virtual int trackPosition(Scrollbar*);
    virtual int trackLength(Scrollbar*);

    virtual int scrollbarThickness(ScrollbarControlSize = RegularScrollbar);

    QStyle* style() const;
};

}
#endif
