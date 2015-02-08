

#include "config.h"
#include "EventNames.h"

namespace WebCore {

#define INITIALIZE_EVENT_NAME(name) \
    , name##Event(#name)
EventNames::EventNames()
    : dummy(0)
DOM_EVENT_NAMES_FOR_EACH(INITIALIZE_EVENT_NAME)
{
}

}
