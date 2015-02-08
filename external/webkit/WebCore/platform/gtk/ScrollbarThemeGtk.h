

#ifndef ScrollbarThemeGtk_h
#define ScrollbarThemeGtk_h

#include "ScrollbarTheme.h"

typedef struct _GtkWidget GtkWidget;
typedef struct _GtkStyle GtkStyle;
typedef struct _GtkContainer GtkContainer;
typedef struct _GtkBorder GtkBorder;

namespace WebCore {

class ScrollbarThemeGtk : public ScrollbarTheme {
public:
    virtual ~ScrollbarThemeGtk();

    virtual int scrollbarThickness(ScrollbarControlSize = RegularScrollbar);
};

}
#endif
