

#ifndef BorderData_h
#define BorderData_h

#include "BorderValue.h"
#include "IntSize.h"
#include "NinePieceImage.h"

namespace WebCore {

class BorderData {
public:
    BorderValue left;
    BorderValue right;
    BorderValue top;
    BorderValue bottom;

    NinePieceImage image;

    IntSize topLeft;
    IntSize topRight;
    IntSize bottomLeft;
    IntSize bottomRight;

    bool hasBorder() const
    {
        bool haveImage = image.hasImage();
        return left.nonZero(!haveImage) || right.nonZero(!haveImage) || top.nonZero(!haveImage) || bottom.nonZero(!haveImage);
    }

    bool hasBorderRadius() const
    {
        if (topLeft.width() > 0)
            return true;
        if (topRight.width() > 0)
            return true;
        if (bottomLeft.width() > 0)
            return true;
        if (bottomRight.width() > 0)
            return true;
        return false;
    }
    
    unsigned short borderLeftWidth() const
    {
        if (!image.hasImage() && (left.style() == BNONE || left.style() == BHIDDEN))
            return 0; 
        return left.width;
    }
    
    unsigned short borderRightWidth() const
    {
        if (!image.hasImage() && (right.style() == BNONE || right.style() == BHIDDEN))
            return 0;
        return right.width;
    }
    
    unsigned short borderTopWidth() const
    {
        if (!image.hasImage() && (top.style() == BNONE || top.style() == BHIDDEN))
            return 0;
        return top.width;
    }
    
    unsigned short borderBottomWidth() const
    {
        if (!image.hasImage() && (bottom.style() == BNONE || bottom.style() == BHIDDEN))
            return 0;
        return bottom.width;
    }
    
    bool operator==(const BorderData& o) const
    {
        return left == o.left && right == o.right && top == o.top && bottom == o.bottom && image == o.image &&
               topLeft == o.topLeft && topRight == o.topRight && bottomLeft == o.bottomLeft && bottomRight == o.bottomRight;
    }
    
    bool operator!=(const BorderData& o) const
    {
        return !(*this == o);
    }
};

} // namespace WebCore

#endif // BorderData_h
