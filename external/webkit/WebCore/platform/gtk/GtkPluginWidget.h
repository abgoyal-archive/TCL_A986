

#ifndef GtkPluginWidget_h
#define GtkPluginWidget_h

#include "Widget.h"

namespace WebCore {
    class GtkPluginWidget : public Widget {
    public:
        GtkPluginWidget(GtkWidget*);
        void invalidateRect(const IntRect&);
        void frameRectsChanged();
        void paint(GraphicsContext*, const IntRect&);
    };
}

#endif
