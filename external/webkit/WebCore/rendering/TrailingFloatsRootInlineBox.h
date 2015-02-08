

#ifndef TrailingFloatsRootInlineBox_h
#define TrailingFloatsRootInlineBox_h

#include "RootInlineBox.h"

namespace WebCore {

class TrailingFloatsRootInlineBox : public RootInlineBox {
public:
    TrailingFloatsRootInlineBox(RenderObject* object) : RootInlineBox(object)
    {
#if ENABLE(SVG)
        setHasVirtualHeight();
#endif
    }

private:
    virtual int virtualHeight() const { return 0; }
};

} // namespace WebCore

#endif // TrailingFloatsRootInlineBox_h
