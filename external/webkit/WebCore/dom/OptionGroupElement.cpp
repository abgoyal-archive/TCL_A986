

#include "config.h"
#include "OptionGroupElement.h"

#include "Element.h"
#include "HTMLNames.h"
#include "HTMLOptGroupElement.h"
#include <wtf/Assertions.h>

#if ENABLE(WML)
#include "WMLOptGroupElement.h"
#include "WMLNames.h"
#endif

namespace WebCore {

OptionGroupElement* toOptionGroupElement(Element* element)
{
    if (element->isHTMLElement() && element->hasTagName(HTMLNames::optgroupTag))
        return static_cast<HTMLOptGroupElement*>(element);

#if ENABLE(WML)
    if (element->isWMLElement() && element->hasTagName(WMLNames::optgroupTag))
        return static_cast<WMLOptGroupElement*>(element);
#endif

    return 0;
}

bool isOptionGroupElement(Element* element)
{
    return element->hasLocalName(HTMLNames::optgroupTag)
#if ENABLE(WML)
        || element->hasLocalName(WMLNames::optgroupTag)
#endif
        ;
}

}
