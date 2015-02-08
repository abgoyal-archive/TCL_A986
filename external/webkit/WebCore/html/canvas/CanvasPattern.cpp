

#include "config.h"
#include "CanvasPattern.h"

#include "ExceptionCode.h"
#include "PlatformString.h"

namespace WebCore {

void CanvasPattern::parseRepetitionType(const String& type, bool& repeatX, bool& repeatY, ExceptionCode& ec)
{
    ec = 0;
    if (type.isEmpty() || type == "repeat") {
        repeatX = true;
        repeatY = true;
        return;
    }
    if (type == "no-repeat") {
        repeatX = false;
        repeatY = false;
        return;
    }
    if (type == "repeat-x") {
        repeatX = true;
        repeatY = false;
        return;
    }
    if (type == "repeat-y") {
        repeatX = false;
        repeatY = true;
        return;
    }
    ec = SYNTAX_ERR;
}

CanvasPattern::CanvasPattern(Image* image, bool repeatX, bool repeatY, bool originClean)
    : m_pattern(Pattern::create(image, repeatX, repeatY))
    , m_originClean(originClean)
{
}

}
