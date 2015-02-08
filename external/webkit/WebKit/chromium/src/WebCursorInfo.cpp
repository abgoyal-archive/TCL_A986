

#include "config.h"
#include "WebCursorInfo.h"

#include "Cursor.h"

using namespace WebCore;

namespace WebKit {

WebCursorInfo::WebCursorInfo(const Cursor& cursor)
{
    type = static_cast<Type>(cursor.impl().type());
    hotSpot = cursor.impl().hotSpot();
    customImage = cursor.impl().customImage();
#ifdef WIN32
    externalHandle = 0;
#endif
}

} // namespace WebKit
