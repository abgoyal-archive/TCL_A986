

#include "config.h"
#include "TextCodec.h"

#include "PlatformString.h"
#include <wtf/StringExtras.h>

namespace WebCore {

TextCodec::~TextCodec()
{
}

int TextCodec::getUnencodableReplacement(unsigned codePoint, UnencodableHandling handling, UnencodableReplacementArray replacement)
{
    switch (handling) {
        case QuestionMarksForUnencodables:
            replacement[0] = '?';
            replacement[1] = 0;
            return 1;
        case EntitiesForUnencodables:
            snprintf(replacement, sizeof(UnencodableReplacementArray), "&#%u;", codePoint);
            return static_cast<int>(strlen(replacement));
        case URLEncodedEntitiesForUnencodables:
            snprintf(replacement, sizeof(UnencodableReplacementArray), "%%26%%23%u%%3B", codePoint);
            return static_cast<int>(strlen(replacement));
    }
    ASSERT_NOT_REACHED();
    replacement[0] = 0;
    return 0;
}

} // namespace WebCore
