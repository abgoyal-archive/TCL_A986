

#include "config.h"
#include "ClipboardChromium.h"

#include "ChromiumDataObject.h"

#include <shlwapi.h>

namespace WebCore {

// Returns true if the specified character is not valid in a file name. This
// is intended for use with removeCharacters.
static bool isInvalidFileCharacter(UChar c)
{
    return (PathGetCharType(c) & (GCT_LFNCHAR | GCT_SHORTCHAR)) == 0;
}

String ClipboardChromium::validateFileName(const String& title, ChromiumDataObject* dataObject)
{
    // Remove any invalid file system characters.
    String result = title.removeCharacters(&isInvalidFileCharacter);
    if (result.length() + dataObject->fileExtension.length() + 1 >= MAX_PATH) {
        if (dataObject->fileExtension.length() + 1 >= MAX_PATH)
            dataObject->fileExtension = "";
        if (result.length() + dataObject->fileExtension.length() + 1 >= MAX_PATH)
            result = result.substring(0, MAX_PATH - dataObject->fileExtension.length() - 1);
    }
    return result;
}

}  // namespace WebCore
