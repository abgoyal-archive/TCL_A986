

#ifndef CSSBorderImageValue_h
#define CSSBorderImageValue_h

#include "CSSValue.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class Rect;

class CSSBorderImageValue : public CSSValue {
public:
    static PassRefPtr<CSSBorderImageValue> create(PassRefPtr<CSSValue> image, PassRefPtr<Rect> sliceRect, int horizontalRule, int verticalRule)
    {
        return adoptRef(new CSSBorderImageValue(image, sliceRect, horizontalRule, verticalRule));
    }

    virtual String cssText() const;

    CSSValue* imageValue() const { return m_image.get(); }

    virtual void addSubresourceStyleURLs(ListHashSet<KURL>&, const CSSStyleSheet*);

    // The border image.
    RefPtr<CSSValue> m_image;

    // These four values are used to make "cuts" in the image.  They can be numbers
    // or percentages.
    RefPtr<Rect> m_imageSliceRect;

    // Values for how to handle the scaling/stretching/tiling of the image slices.
    int m_horizontalSizeRule; // Rule for how to adjust the widths of the top/middle/bottom
    int m_verticalSizeRule; // Rule for how to adjust the heights of the left/middle/right

private:
    CSSBorderImageValue(PassRefPtr<CSSValue> image, PassRefPtr<Rect> sliceRect, int horizontalRule, int verticalRule);
};

} // namespace WebCore

#endif // CSSBorderImageValue_h
