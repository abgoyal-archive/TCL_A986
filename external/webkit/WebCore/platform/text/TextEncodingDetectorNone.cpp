

#include "config.h"
#include "TextEncodingDetector.h"

#include "TextEncoding.h"

namespace WebCore {

bool detectTextEncoding(const char*, size_t, const char*, TextEncoding* detectedEncoding)
{
    *detectedEncoding = TextEncoding();
    return false;
}

}
