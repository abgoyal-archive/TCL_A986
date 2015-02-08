

#include "config.h"
#include "RenderWordBreak.h"

#include "HTMLElement.h"

namespace WebCore {

RenderWordBreak::RenderWordBreak(HTMLElement* element)
    : RenderText(element, StringImpl::empty())
{
}

const char* RenderWordBreak::renderName() const
{
    return "RenderWordBreak";
}

bool RenderWordBreak::isWordBreak() const
{
    return true;
}

}
