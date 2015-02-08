

#include "config.h"
#include "AnimationList.h"

namespace WebCore {

#define FILL_UNSET_PROPERTY(test, propGet, propSet) \
for (i = 0; i < size() && animation(i)->test(); ++i) { } \
if (i < size() && i != 0) { \
    for (size_t j = 0; i < size(); ++i, ++j) \
        animation(i)->propSet(animation(j)->propGet()); \
}

void AnimationList::fillUnsetProperties()
{
    size_t i;
    FILL_UNSET_PROPERTY(isDelaySet, delay, setDelay);
    FILL_UNSET_PROPERTY(isDirectionSet, direction, setDirection);
    FILL_UNSET_PROPERTY(isDurationSet, duration, setDuration);
    FILL_UNSET_PROPERTY(isIterationCountSet, iterationCount, setIterationCount);
    FILL_UNSET_PROPERTY(isPlayStateSet, playState, setPlayState);
    FILL_UNSET_PROPERTY(isNameSet, name, setName);
    FILL_UNSET_PROPERTY(isTimingFunctionSet, timingFunction, setTimingFunction);
    FILL_UNSET_PROPERTY(isPropertySet, property, setProperty);
}

bool AnimationList::operator==(const AnimationList& o) const
{
    if (size() != o.size())
        return false;
    for (size_t i = 0; i < size(); ++i)
        if (*animation(i) != *o.animation(i))
            return false;
    return true;
}

} // namespace WebCore
