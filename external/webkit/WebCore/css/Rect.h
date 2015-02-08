

#ifndef Rect_h
#define Rect_h

#include "CSSPrimitiveValue.h"
#include <wtf/RefPtr.h>

namespace WebCore {

    class RectBase {
    public:
        CSSPrimitiveValue* top() const { return m_top.get(); }
        CSSPrimitiveValue* right() const { return m_right.get(); }
        CSSPrimitiveValue* bottom() const { return m_bottom.get(); }
        CSSPrimitiveValue* left() const { return m_left.get(); }

        void setTop(PassRefPtr<CSSPrimitiveValue> top) { m_top = top; }
        void setRight(PassRefPtr<CSSPrimitiveValue> right) { m_right = right; }
        void setBottom(PassRefPtr<CSSPrimitiveValue> bottom) { m_bottom = bottom; }
        void setLeft(PassRefPtr<CSSPrimitiveValue> left) { m_left = left; }

    protected:
        RectBase() { }
        ~RectBase() { }

    private:
        RefPtr<CSSPrimitiveValue> m_top;
        RefPtr<CSSPrimitiveValue> m_right;
        RefPtr<CSSPrimitiveValue> m_bottom;
        RefPtr<CSSPrimitiveValue> m_left;
    };

    class Rect : public RectBase, public RefCounted<Rect> {
    public:
        static PassRefPtr<Rect> create() { return adoptRef(new Rect); }

    private:
        Rect() { }
    };

} // namespace WebCore

#endif // Rect_h
