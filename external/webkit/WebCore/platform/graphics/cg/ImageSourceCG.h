

#ifndef ImageSourceCG_h
#define ImageSourceCG_h

#include "ImageSource.h"

namespace WebCore {

class String;

String preferredExtensionForImageSourceType(const String& type);

String MIMETypeForImageSourceType(const String& type);

#if !PLATFORM(MAC)
size_t sharedBufferGetBytesAtPosition(void* info, void* buffer, off_t position, size_t count);
#endif

}

#endif // ImageSourceCG_h
