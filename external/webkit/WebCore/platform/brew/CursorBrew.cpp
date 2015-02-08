

#define LOG_TAG "WebCore"

#include "config.h"
#include "Cursor.h"

#include "NotImplemented.h"

#include <wtf/StdLibExtras.h>

namespace WebCore {

Cursor::Cursor(Image*, const IntPoint&)
{
    notImplemented();
}

Cursor::Cursor(const Cursor&)
{
    notImplemented();
}

Cursor::~Cursor()
{
    notImplemented();
}

Cursor& Cursor::operator=(const Cursor&)
{
    notImplemented();
    return *this;
}

static inline Cursor& dummyCursor()
{
    DEFINE_STATIC_LOCAL(Cursor, c, ());
    return c;
}

const Cursor& pointerCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& crossCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& handCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& moveCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& iBeamCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& waitCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& helpCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& eastResizeCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& northResizeCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& northEastResizeCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& northWestResizeCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& southResizeCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& southEastResizeCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& southWestResizeCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& westResizeCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& northSouthResizeCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& eastWestResizeCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& northEastSouthWestResizeCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& northWestSouthEastResizeCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& columnResizeCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& rowResizeCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& verticalTextCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& cellCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& contextMenuCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& noDropCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& copyCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& progressCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& aliasCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& noneCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& notAllowedCursor()
{
    return dummyCursor();
}

const Cursor& zoomInCursor()
{
    return dummyCursor();
}

const Cursor& zoomOutCursor()
{
    return dummyCursor();
}

const Cursor& middlePanningCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& eastPanningCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& northPanningCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& northEastPanningCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& northWestPanningCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& southPanningCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& southEastPanningCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& southWestPanningCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& westPanningCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& grabCursor()
{
    notImplemented();
    return dummyCursor();
}

const Cursor& grabbingCursor()
{
    notImplemented();
    return dummyCursor();
}

} // namespace WebCore
