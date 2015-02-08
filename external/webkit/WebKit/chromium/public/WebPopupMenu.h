

#ifndef WebPopupMenu_h
#define WebPopupMenu_h

#include "WebCommon.h"
#include "WebWidget.h"

namespace WebKit {

class WebWidgetClient;

class WebPopupMenu : public WebWidget {
public:
    WEBKIT_API static WebPopupMenu* create(WebWidgetClient*);
};

} // namespace WebKit

#endif
