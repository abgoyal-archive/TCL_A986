

#ifndef WebCursorInfo_h
#define WebCursorInfo_h

#include "WebImage.h"
#include "WebPoint.h"

#if WEBKIT_IMPLEMENTATION
namespace WebCore { class Cursor; }
#endif

#ifdef WIN32
typedef struct HICON__* HICON;
typedef HICON HCURSOR;
#endif

namespace WebKit {

struct WebCursorInfo {
    enum Type {
        TypePointer,
        TypeCross,
        TypeHand,
        TypeIBeam,
        TypeWait,
        TypeHelp,
        TypeEastResize,
        TypeNorthResize,
        TypeNorthEastResize,
        TypeNorthWestResize,
        TypeSouthResize,
        TypeSouthEastResize,
        TypeSouthWestResize,
        TypeWestResize,
        TypeNorthSouthResize,
        TypeEastWestResize,
        TypeNorthEastSouthWestResize,
        TypeNorthWestSouthEastResize,
        TypeColumnResize,
        TypeRowResize,
        TypeMiddlePanning,
        TypeEastPanning,
        TypeNorthPanning,
        TypeNorthEastPanning,
        TypeNorthWestPanning,
        TypeSouthPanning,
        TypeSouthEastPanning,
        TypeSouthWestPanning,
        TypeWestPanning,
        TypeMove,
        TypeVerticalText,
        TypeCell,
        TypeContextMenu,
        TypeAlias,
        TypeProgress,
        TypeNoDrop,
        TypeCopy,
        TypeNone,
        TypeNotAllowed,
        TypeZoomIn,
        TypeZoomOut,
        TypeCustom
    };

    Type type;
    WebPoint hotSpot;
    WebImage customImage;

#ifdef WIN32
    // On Windows, TypeCustom may alternatively reference an externally
    // defined HCURSOR.  If type is TypeCustom and externalHandle is non-
    // null, then customData should be ignored.  The WebCursorInfo is not
    // responsible for managing the lifetime of this cursor handle.
    HCURSOR externalHandle;
#endif

    explicit WebCursorInfo(Type type = TypePointer)
        : type(type)
    {
#ifdef WIN32
        externalHandle = 0;
#endif
    }

#if WEBKIT_IMPLEMENTATION
    explicit WebCursorInfo(const WebCore::Cursor&);
#endif
};

} // namespace WebKit

#endif
