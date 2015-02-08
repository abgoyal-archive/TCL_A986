

#ifndef ScrollbarGtk_h
#define ScrollbarGtk_h

#include "Scrollbar.h"
#include <wtf/PassRefPtr.h>

typedef struct _GtkAdjustment GtkAdjustment;

namespace WebCore {

class ScrollbarGtk : public Scrollbar {
public:
    friend class Scrollbar;
    friend class ScrollView;
    ~ScrollbarGtk();

    virtual void setFrameRect(const IntRect&);
    virtual void paint(GraphicsContext*, const IntRect&);
    
    virtual bool handleMouseMoveEvent(const PlatformMouseEvent&) { return false; }
    virtual bool handleMouseOutEvent(const PlatformMouseEvent&) { return false; }
    virtual bool handleMousePressEvent(const PlatformMouseEvent&) { return false; }
    virtual bool handleMouseReleaseEvent(const PlatformMouseEvent&) { return false; }

    virtual void setEnabled(bool);

    virtual void frameRectsChanged();

protected:
    static PassRefPtr<ScrollbarGtk> createScrollbar(ScrollbarClient* client, ScrollbarOrientation orientation, GtkAdjustment*);

    ScrollbarGtk(ScrollbarClient*, ScrollbarOrientation, ScrollbarControlSize);
    ScrollbarGtk(ScrollbarClient*, ScrollbarOrientation, GtkAdjustment*);

    virtual void updateThumbPosition();
    virtual void updateThumbProportion();

    void detachAdjustment();
    void attachAdjustment(GtkAdjustment*);
private:
    static void gtkValueChanged(GtkAdjustment*, ScrollbarGtk*);
    IntPoint getLocationInParentWindow(const IntRect&);
    GtkAdjustment* m_adjustment;
};

}

#endif // ScrollbarGtk_h
