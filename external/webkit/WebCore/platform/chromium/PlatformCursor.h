

#ifndef PlatformCursor_h
#define PlatformCursor_h

#include "Image.h"
#include "IntPoint.h"
#include "RefPtr.h"

namespace WebCore {

    class PlatformCursor {
    public:
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

        // Cursor.h assumes that it can initialize us to 0.
        explicit PlatformCursor(int type = 0) : m_type(TypePointer) {}
        
        PlatformCursor(Type type) : m_type(type) {}

        PlatformCursor(Image* image, const IntPoint& hotSpot)
            : m_image(image)
            , m_hotSpot(hotSpot)
            , m_type(TypeCustom) {}

        PassRefPtr<Image> customImage() const { return m_image; }
        const IntPoint& hotSpot() const { return m_hotSpot; }
        Type type() const { return m_type; }

    private:
        RefPtr<Image> m_image;
        IntPoint m_hotSpot;
        Type m_type;
    };

} // namespace WebCore

#endif
