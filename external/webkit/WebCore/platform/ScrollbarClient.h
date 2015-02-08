

#ifndef ScrollbarClient_h
#define ScrollbarClient_h

#include "IntRect.h"
#include "Scrollbar.h"
#include <wtf/Vector.h>

namespace WebCore {

class ScrollbarClient {
public:
    virtual ~ScrollbarClient() {}
    virtual void valueChanged(Scrollbar*) = 0;

    virtual void invalidateScrollbarRect(Scrollbar*, const IntRect&) = 0;

    virtual bool isActive() const = 0;
    
    virtual bool scrollbarCornerPresent() const = 0;

    virtual void getTickmarks(Vector<IntRect>&) const { }

    // Convert points and rects between the scrollbar and its containing view.
    // The client needs to implement these in order to be aware of layout effects
    // like CSS transforms.
    virtual IntRect convertFromScrollbarToContainingView(const Scrollbar* scrollbar, const IntRect& scrollbarRect) const
    {
        return scrollbar->Widget::convertToContainingView(scrollbarRect);
    }
    
    virtual IntRect convertFromContainingViewToScrollbar(const Scrollbar* scrollbar, const IntRect& parentRect) const
    {
        return scrollbar->Widget::convertFromContainingView(parentRect);
    }
    
    virtual IntPoint convertFromScrollbarToContainingView(const Scrollbar* scrollbar, const IntPoint& scrollbarPoint) const
    {
        return scrollbar->Widget::convertToContainingView(scrollbarPoint);
    }
    
    virtual IntPoint convertFromContainingViewToScrollbar(const Scrollbar* scrollbar, const IntPoint& parentPoint) const
    {
        return scrollbar->Widget::convertFromContainingView(parentPoint);
    }
};

}
#endif
