

#include "config.h"

#if ENABLE(VIDEO)
#include "HTMLAudioElement.h"

#include "HTMLNames.h"

namespace WebCore {

using namespace HTMLNames;

HTMLAudioElement::HTMLAudioElement(const QualifiedName& tagName, Document* doc)
    : HTMLMediaElement(tagName, doc)
{
    ASSERT(hasTagName(audioTag));
}

}
#endif
