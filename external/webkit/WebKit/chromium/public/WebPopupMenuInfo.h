

#ifndef WebPopupMenuInfo_h
#define WebPopupMenuInfo_h

#include "WebMenuItemInfo.h"
#include "WebVector.h"

namespace WebKit {

// Describes the contents of a popup menu.
struct WebPopupMenuInfo {
    // FIXME: migrate clients to WebMenuItemInfo and remove this temporary Item typedef.
    typedef WebMenuItemInfo Item;
    int itemHeight;
    int selectedIndex;
    WebVector<WebMenuItemInfo> items;
};

} // namespace WebKit

#endif
