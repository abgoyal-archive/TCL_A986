

#ifndef RenderWordBreak_h
#define RenderWordBreak_h

#include "RenderText.h"

namespace WebCore {

class HTMLElement;

class RenderWordBreak : public RenderText {
public:
    RenderWordBreak(HTMLElement*);

    virtual const char* renderName() const;
    virtual bool isWordBreak() const;
};

}

#endif
