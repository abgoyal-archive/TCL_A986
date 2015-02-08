

#ifndef WebMenuItemInfo_h
#define WebMenuItemInfo_h

#include "WebCommon.h"
#include "WebString.h"
#include "WebVector.h"

namespace WebKit {

struct WebMenuItemInfo {
    enum Type {
        Option,
        CheckableOption,
        Group,
        Separator,
    };
    WebString label;
    Type type;
    unsigned action;
    bool enabled;
    bool checked;
};

} // namespace WebKit

#endif
