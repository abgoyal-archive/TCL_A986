
#include "unicode/utypes.h"

#include "ucreader.h"
#include "gsupport.h"
#include "UnicodeReader.h"

U_CDECL_BEGIN

const UChar *uc_readFile(const char *fileName, gs_guiSupport *guiSupport, int32_t *charCount)
{
    return UnicodeReader::readFile(fileName, (GUISupport *) guiSupport, *charCount);
}

U_CDECL_END
