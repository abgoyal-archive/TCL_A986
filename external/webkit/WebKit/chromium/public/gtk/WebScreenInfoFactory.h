

#ifndef WebScreenInfoFactory_h
#define WebScreenInfoFactory_h

#include "../WebCommon.h"

typedef struct _GtkWidget GtkWidget;

namespace WebKit {

struct WebScreenInfo;

class WebScreenInfoFactory {
public:
    WEBKIT_API static WebScreenInfo screenInfo(GtkWidget*);
};

} // namespace WebKit

#endif
