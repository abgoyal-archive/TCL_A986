

#ifndef AccessibleImage_h
#define AccessibleImage_h

#include "AccessibleBase.h"

class AccessibleImage : public AccessibleBase
{
public:
    AccessibleImage(WebCore::AccessibilityObject*);
    virtual ~AccessibleImage() { }

private:
    virtual WebCore::String name() const;
};

#endif // AccessibleImage_h
